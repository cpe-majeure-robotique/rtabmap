/*
Copyright (c) 2025 Felix Toft
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/core/odometry/OdometryCuVSLAM.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include <cmath>

#ifdef RTABMAP_CUVSLAM
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/util3d_transforms.h"
#include "CuVSLAMCovariance.h"
#include <cuvslam/cuvslam2.h>
#include <cuvslam/ground_constraint2.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <cuda_runtime.h>
#include <exception>

// ============================================================================
// Coordinate System Transformation Constants
// Based on Isaac ROS implementation:
// Source: isaac_ros_visual_slam/include/isaac_ros_visual_slam/impl/cuvslam_ros_conversion.hpp
// ============================================================================

// cuVSLAM v15 uses OpenCV/optical coordinates (x-right, y-down, z-forward).
// Transform converting from canonical ROS frame (x-forward, y-left, z-up)
// to cuVSLAM/optical frame.
const rtabmap::Transform cuvslam_pose_canonical(
    0, -1, 0, 0,
    0, 0, -1, 0,
    1, 0, 0, 0
);

// Transformation converting from cuVSLAM/optical frame to canonical ROS frame.
const rtabmap::Transform canonical_pose_cuvslam = cuvslam_pose_canonical.inverse();

const rtabmap::Transform cuvslam_pose_optical = rtabmap::Transform::getIdentity();
const rtabmap::Transform optical_pose_cuvslam = cuvslam_pose_optical.inverse();


// ============================================================================
// Forward Declarations
// ============================================================================

namespace rtabmap {

enum class CuVSLAMInputMode
{
    Stereo,
    RGBD
};

bool initializeCuVSLAM(const SensorData & data, 
                       std::unique_ptr<cuvslam::Odometry> & cuvslam_handle,
                       std::unique_ptr<cuvslam::GroundConstraint> & ground_constraint_handle,
                       bool planar_constraints,
                       int multicam_mode,
                       CuVSLAMInputMode input_mode,
                       std::vector<uint8_t *> & gpu_left_image_data,
                       std::vector<uint8_t *> & gpu_right_image_data,
                       std::vector<uint8_t *> & gpu_depth_image_data,
                       std::vector<size_t> & gpu_left_image_sizes,
                       std::vector<size_t> & gpu_right_image_sizes,
                       std::vector<size_t> & gpu_depth_image_sizes,
                       cuvslam::Rig & cuvslam_rig,
                       cudaStream_t & cuda_stream);

cuvslam::Odometry::Config CreateConfiguration(const SensorData & data, int multicam_mode, CuVSLAMInputMode input_mode);

bool prepareImages(const SensorData & data, 
                    std::vector<cuvslam::Image> & cuvslam_images,
                    std::vector<cuvslam::Image> & cuvslam_depths,
                    CuVSLAMInputMode input_mode,
                    std::vector<uint8_t *> & gpu_left_image_data,
                    std::vector<uint8_t *> & gpu_right_image_data,
                    std::vector<uint8_t *> & gpu_depth_image_data,
                    std::vector<size_t> & gpu_left_image_sizes,
                    std::vector<size_t> & gpu_right_image_sizes,
                    std::vector<size_t> & gpu_depth_image_sizes,
                    cudaStream_t & cuda_stream);

// ============================================================================
// Transform Conversion Functions and Misc Helpers
// ============================================================================

cuvslam::Pose TocuVSLAMPose(const Transform & rtabmap_transform)
{
  cuvslam::Pose cuvslam_pose;
  const Eigen::Quaternionf q = rtabmap_transform.getQuaternionf();
  cuvslam_pose.rotation = {q.x(), q.y(), q.z(), q.w()};
  cuvslam_pose.translation = {rtabmap_transform.x(), rtabmap_transform.y(), rtabmap_transform.z()};
  return cuvslam_pose;
}

Transform FromcuVSLAMPose(const cuvslam::Pose & cuvslam_pose)
{
  const auto & q = cuvslam_pose.rotation;
  const auto & t = cuvslam_pose.translation;
  return Transform(t[0], t[1], t[2], q[0], q[1], q[2], q[3]);
}

} // namespace rtabmap

#endif

// ============================================================================
// OdometryCuVSLAM Class Implementation
// ============================================================================

namespace rtabmap {

OdometryCuVSLAM::OdometryCuVSLAM(const ParametersMap & parameters) :
    Odometry(parameters)
#ifdef RTABMAP_CUVSLAM
    ,
    cuvslam_handle_(nullptr),
    ground_constraint_handle_(nullptr),
    initialized_(false),
    lost_(false),
    tracking_(false),
    planar_constraints_(false),
    multicam_mode_(0),
    previous_pose_(Transform::getIdentity()),
    use_raw_covariance_(false),
    covariance_position_scale_(1.0),
    covariance_orientation_scale_(1.0),
    covariance_position_floor_(1e-6),
    covariance_orientation_floor_(1e-6),
    covariance_position_ceiling_(100.0),
    covariance_orientation_ceiling_(9.8696),
    covariance_fallback_position_(0.25),
    covariance_fallback_orientation_(0.1),
    covariance_decrease_smoothing_(0.9),
    min_landmarks_threshold_(0),
    previous_covariance_(),
    gpu_left_image_data_(),
    gpu_right_image_data_(),
    gpu_depth_image_data_(),
    gpu_left_image_sizes_(),
    gpu_right_image_sizes_(),
    gpu_depth_image_sizes_(),
    cuda_stream_(nullptr)
#endif
{
#ifdef RTABMAP_CUVSLAM
    // Reg/Force3DoF is already enforced by the Odometry base class after
    // computeTransform() returns. Do not map it to cuVSLAM GroundConstraint:
    // with cuVSLAM v15/optical coordinates it can constrain the wrong axes.
    planar_constraints_ = false;
	Parameters::parse(parameters, Parameters::kOdomCuVSLAMMulticamMode(), multicam_mode_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMUseRawCovariance(), use_raw_covariance_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMCovariancePositionScale(), covariance_position_scale_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMCovarianceOrientationScale(), covariance_orientation_scale_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMCovariancePositionFloor(), covariance_position_floor_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMCovarianceOrientationFloor(), covariance_orientation_floor_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMCovariancePositionCeiling(), covariance_position_ceiling_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMCovarianceOrientationCeiling(), covariance_orientation_ceiling_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMCovarianceFallbackPosition(), covariance_fallback_position_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMCovarianceFallbackOrientation(), covariance_fallback_orientation_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMCovarianceDecreaseSmoothing(), covariance_decrease_smoothing_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMMinLandmarks(), min_landmarks_threshold_);
    UASSERT(multicam_mode_ >= 0 && multicam_mode_ <= 2);
    UASSERT(covariance_position_scale_ > 0.0 && covariance_orientation_scale_ > 0.0);
    UASSERT(covariance_position_floor_ > 0.0 && covariance_orientation_floor_ > 0.0);
    UASSERT(covariance_position_ceiling_ >= covariance_position_floor_);
    UASSERT(covariance_orientation_ceiling_ >= covariance_orientation_floor_);
    UASSERT(covariance_fallback_position_ >= covariance_position_floor_ && covariance_fallback_position_ <= covariance_position_ceiling_);
    UASSERT(covariance_fallback_orientation_ >= covariance_orientation_floor_ && covariance_fallback_orientation_ <= covariance_orientation_ceiling_);
    UASSERT(covariance_decrease_smoothing_ >= 0.0 && covariance_decrease_smoothing_ < 1.0);
    UASSERT(min_landmarks_threshold_ >= 0);
    UINFO("%s=%d", Parameters::kOdomCuVSLAMMulticamMode().c_str(), multicam_mode_);
    UINFO("cuVSLAM covariance: raw=%s, variance scale=%g translation / %g rotation, floor=%g m^2 / %g rad^2, fallback=%g m^2 / %g rad^2, decrease smoothing=%g",
          use_raw_covariance_ ? "true" : "false",
          covariance_position_scale_,
          covariance_orientation_scale_,
          covariance_position_floor_,
          covariance_orientation_floor_,
          covariance_fallback_position_,
          covariance_fallback_orientation_,
          covariance_decrease_smoothing_);
    // Warm up GPU and create CUDA context before tracker initialization
    // Supposedly this will speed up the tracker initialization
    try {
        cuvslam::WarmUpGPU();
    }
    catch(const std::exception & e) {
        UERROR("cuVSLAM GPU warmup failed: %s", e.what());
    }
#endif
}

OdometryCuVSLAM::~OdometryCuVSLAM()
{
#ifdef RTABMAP_CUVSLAM
    cuvslam_handle_.reset();
    ground_constraint_handle_.reset();
    
    // Clean up GPU memory
    for(uint8_t * gpu_ptr : gpu_left_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    for(uint8_t * gpu_ptr : gpu_right_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    for(uint8_t * gpu_ptr : gpu_depth_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    if(cuda_stream_) {
        cudaStreamDestroy(cuda_stream_);
        cuda_stream_ = nullptr;
    }
#endif
}

void OdometryCuVSLAM::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);
   
#ifdef RTABMAP_CUVSLAM
    this->cleanupCuVSLAMResources();
#endif
}

void OdometryCuVSLAM::cleanupCuVSLAMResources()
{
#ifdef RTABMAP_CUVSLAM
    cuvslam_handle_.reset();
    ground_constraint_handle_.reset();
    
    // Clean up GPU memory
    for(uint8_t * gpu_ptr : gpu_left_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    gpu_left_image_data_.clear();
    for(uint8_t * gpu_ptr : gpu_right_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    gpu_right_image_data_.clear();
    for(uint8_t * gpu_ptr : gpu_depth_image_data_) {
        if(gpu_ptr) {
            cudaFree(gpu_ptr);
        }
    }
    gpu_depth_image_data_.clear();
    if(cuda_stream_) {
        cudaStreamDestroy(cuda_stream_);
        cuda_stream_ = nullptr;
    }
    
    // Reset our internal state variables
    gpu_left_image_sizes_.clear();
    gpu_right_image_sizes_.clear();
    gpu_depth_image_sizes_.clear();
    cuvslam_rig_ = cuvslam::Rig();
    initialized_ = false;
    lost_ = false;
    tracking_ = false;
    previous_pose_ = Transform::getIdentity();
    previous_covariance_ = cv::Mat();
#endif
}

Transform OdometryCuVSLAM::computeTransform(
    SensorData & data,
    const Transform & guess,
    OdometryInfo * info)
{    
#ifdef RTABMAP_CUVSLAM
    UTimer timer;
    (void)guess; // cuVSLAM 15 no longer accepts an external pose prediction.

    UDEBUG("=== computeTransform ENTRY === lost_=%s, tracking_=%s, initialized_=%s",
          lost_ ? "true" : "false",
          tracking_ ? "true" : "false",
          initialized_ ? "true" : "false");

    // Keep feeding images after a transient tracking loss. cuVSLAM can recover
    // by itself; latching here used to turn a single missed pose into an
    // RTAB-Map reset even when the following frames were valid.

    const bool has_stereo_input =
            !data.imageRaw().empty() &&
            !data.rightRaw().empty() &&
            !data.stereoCameraModels().empty();
    const bool has_rgbd_input =
            !data.imageRaw().empty() &&
            !data.depthRaw().empty() &&
            !data.cameraModels().empty();

    if(!has_stereo_input && !has_rgbd_input)
    {
        UERROR("cuVSLAM odometry requires either stereo input (left+right+stereo model) or RGB-D input (rgb+depth+camera model). Left/RGB: %s, Right: %s, Depth: %s, stereo models: %d, camera models: %d",
               data.imageRaw().empty() ? "empty" : "ok",
               data.rightRaw().empty() ? "empty" : "ok",
               data.depthRaw().empty() ? "empty" : "ok",
               static_cast<int>(data.stereoCameraModels().size()),
               static_cast<int>(data.cameraModels().size()));
        return Transform();
    }

    const CuVSLAMInputMode input_mode = has_stereo_input ? CuVSLAMInputMode::Stereo : CuVSLAMInputMode::RGBD;

    if(input_mode == CuVSLAMInputMode::RGBD && data.cameraModels().size() != 1)
    {
        UERROR("cuVSLAM RGB-D odometry supports exactly one RGB-D camera model, got %d", static_cast<int>(data.cameraModels().size()));
        return Transform();
    }

    // Initialize cuVSLAM tracker on first frame
    if(!initialized_)
    {   
        if(!initializeCuVSLAM(
            data, 
            cuvslam_handle_,
            ground_constraint_handle_,
            planar_constraints_,
            multicam_mode_,
            input_mode,
            gpu_left_image_data_,
            gpu_right_image_data_,
            gpu_depth_image_data_,
            gpu_left_image_sizes_,
            gpu_right_image_sizes_,
            gpu_depth_image_sizes_,
            cuvslam_rig_,
            cuda_stream_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return Transform();
        }
    }
        
    // Prepare images for cuVSLAM
    std::vector<cuvslam::Image> cuvslam_image_objects;
    std::vector<cuvslam::Image> cuvslam_depth_objects;
    if(!prepareImages(
        data,
        cuvslam_image_objects,
        cuvslam_depth_objects,
        input_mode,
        gpu_left_image_data_,
        gpu_right_image_data_,
        gpu_depth_image_data_,
        gpu_left_image_sizes_,
        gpu_right_image_sizes_,
        gpu_depth_image_sizes_,
        cuda_stream_))
    {
        UERROR("Failed to prepare images for cuVSLAM");
        return Transform();
    }
    
    // Not using the IMU yet
    if(!data.imu().empty())
    {
        UWARN("IMU data available but processing not implemented yet");
    }
    
    // Validate images and tracker status
    if(cuvslam_image_objects.empty()) {
        UERROR("No images prepared for cuVSLAM tracking");
        return Transform();
    }
    if(!cuvslam_handle_) {
        UERROR("cuVSLAM tracker is null! initialized_: %s", initialized_ ? "true" : "false");
        return Transform();
    }

    cuvslam::PoseEstimate vo_pose_estimate;
    try
    {
        vo_pose_estimate = cuvslam_handle_->Track(cuvslam_image_objects, {}, cuvslam_depth_objects);
    }
    catch(const std::exception & e)
    {
        if(info)
        {
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
            info->timeEstimation = timer.ticks();
        }
        lost_ = tracking_;
        UWARN("cuVSLAM tracking exception: %s", e.what());
        return Transform();
    }

    if(!vo_pose_estimate.world_from_rig.has_value())
    {
        if(info)
        {
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
            info->timeEstimation = timer.ticks();
        }
        UWARN("cuVSLAM tracking failed: pose estimate is empty");
        lost_ = tracking_;
        return Transform();
    }

    cuvslam::PoseWithCovariance & world_from_rig = *vo_pose_estimate.world_from_rig;

    const CuVSLAMCovarianceConfig covariance_config = {
        use_raw_covariance_,
        covariance_position_scale_,
        covariance_orientation_scale_,
        covariance_position_floor_,
        covariance_orientation_floor_,
        covariance_position_ceiling_,
        covariance_orientation_ceiling_,
        covariance_fallback_position_,
        covariance_fallback_orientation_,
        covariance_decrease_smoothing_};
    bool covariance_repaired = false;
    cv::Mat covMat = convertCuVSLAMCovariance(
            world_from_rig.covariance,
            covariance_config,
            previous_covariance_,
            &covariance_repaired);
    if(covariance_repaired)
    {
        UDEBUG("cuVSLAM returned a non-finite, non-positive or non-PSD covariance; using a conservative repaired covariance for this frame.");
    }
    
    // Apply ground constraint
    if(planar_constraints_) {
        try {
            ground_constraint_handle_->AddNextPose(world_from_rig.pose);
            world_from_rig.pose = ground_constraint_handle_->GetPoseOnGround();
        }
        catch(const std::exception & e) {
            UERROR("Failed to apply cuVSLAM ground constraint: %s", e.what());
            return Transform();
        }
    }

    // Convert cuVSLAM absolute pose to incremental RTAB-Map Transform
    Transform current_pose = FromcuVSLAMPose(world_from_rig.pose);
    UASSERT(!previous_pose_.isNull());
    Transform transform = previous_pose_.inverse() * current_pose;

    // A high but finite covariance means low confidence, not tracking loss.
    // Let the EKF down-weight it and RTAB-Map weaken the corresponding link.
    tracking_ = true;
    lost_ = false;

    if(info)
    {
        info->reg.covariance = covMat;
        info->timeEstimation = timer.ticks();
    }

    int landmarks_num = 0;
    landmarks_.clear();
    try {
        landmarks_ = cuvslam_handle_->GetLastLandmarks();
        landmarks_num = landmarks_.size();
    }
    catch(const std::exception & e) {
        UWARN("Failed to get cuVSLAM landmarks: %s", e.what());
    }

    // Fill info with visualization data. Clear exports every frame so a
    // transient API error cannot leak stale quality data into RTAB-Map.
    observations_.clear();
    if(info) {
        const size_t camera_models_count =
                input_mode == CuVSLAMInputMode::Stereo ?
                data.stereoCameraModels().size() :
                data.cameraModels().size();

        if(camera_models_count==1) {
            info->type = kTypeF2F;

            try {
                observations_ = cuvslam_handle_->GetLastObservations(0);
            }
            catch(const std::exception & e) {
                UWARN("Failed to get cuVSLAM observations: %s", e.what());
            }
            if(!observations_.empty()) {
                info->newCorners.reserve(observations_.size());
                for(size_t i = 0; i < observations_.size(); ++i)
                {
                    const cuvslam::Observation & observation = observations_[i];
                    info->newCorners.emplace_back(observation.u, observation.v);
                }
            }
        }
        else {
            info->type = kTypeF2M;
        }
        
        std::vector<Transform> local_transform_inv(camera_models_count);
        for(size_t i=0; i<camera_models_count; ++i) {
            local_transform_inv[i] =
                    input_mode == CuVSLAMInputMode::Stereo ?
                    data.stereoCameraModels()[i].localTransform().inverse() :
                    data.cameraModels()[i].localTransform().inverse();
        }
        int image_width = data.imageRaw().cols / camera_models_count;
        if(!landmarks_.empty()) {
            Transform absolute_pose = this->getPose() * transform;
            for(size_t i = 0; i < landmarks_.size(); ++i)
            {
                const cuvslam::Landmark & landmark = landmarks_[i];
                cv::Point3f pt(landmark.coords[0], landmark.coords[1], landmark.coords[2]);
                info->localMap.insert(std::make_pair(landmark.id, util3d::transformPoint(pt, absolute_pose)));
                if(camera_models_count > 1 || input_mode == CuVSLAMInputMode::RGBD) {
                    for(size_t i=0; i<camera_models_count; ++i) {
                        cv::Point3f pt_in_cam = util3d::transformPoint(pt, local_transform_inv[i]);
                        float u,v;
                        if(pt_in_cam.z > 0)
                        {
                            if(input_mode == CuVSLAMInputMode::Stereo)
                            {
                                data.stereoCameraModels()[i].left().reproject(pt_in_cam.x, pt_in_cam.y, pt_in_cam.z, u, v);
                            }
                            else
                            {
                                data.cameraModels()[i].reproject(pt_in_cam.x, pt_in_cam.y, pt_in_cam.z, u, v);
                            }
                            const bool in_frame =
                                    input_mode == CuVSLAMInputMode::Stereo ?
                                    data.stereoCameraModels()[i].left().inFrame(u,v) :
                                    data.cameraModels()[i].inFrame(u,v);
                            if(in_frame)
                            {
                                info->words.insert(std::make_pair(landmark.id, cv::KeyPoint(u + i*image_width, v, 3)));
                                info->reg.inliersIDs.push_back(landmark.id);
                                break;
                            }

                            // Update landmarks number based on which landmarks were successfully reprojected in the current frame
                            landmarks_num = info->words.size();
                        }
                    }
                }
            }
        }

        // If we are in a multi-camera setup and successfully reprojected landmarks into camera frames,
        // use the number of successfully reprojected landmarks instead of the raw cuVSLAM landmark count.
        if(camera_models_count > 1 || input_mode == CuVSLAMInputMode::RGBD) {
            landmarks_num = (int)info->words.size();
        }

        info->features = observations_.empty() ? landmarks_.size() : observations_.size();
        info->localMapSize = landmarks_.size();
        info->reg.matches = info->features;
        info->reg.inliers = landmarks_num;
        info->reg.inliersRatio = info->reg.matches > 0 ? float(info->reg.inliers) / float(info->reg.matches) : 0.0f;
    }
    
    // Check if we have enough features to start tracking. Otherwise we are lost.
    if(min_landmarks_threshold_ > 0 && landmarks_num < min_landmarks_threshold_ && !initialized_) {
        if(info) {
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
            info->timeEstimation = timer.ticks();
        }
        // Free GPU resources and reset state. Prevent memory leaks on init loops.
        cleanupCuVSLAMResources();
        lost_ = true;
        tracking_ = false;
        initialized_ = false;
        return Transform();
    } else {
        initialized_ = true;
    }

    previous_pose_ = current_pose;
    previous_covariance_ = covMat;
    return transform;
#else
    UERROR("cuVSLAM support not compiled in RTAB-Map");\
    return Transform();
#endif
   
}

#ifdef RTABMAP_CUVSLAM

// ============================================================================
// cuVSLAM Initialization and Configuration
// ============================================================================

bool initializeCuVSLAM(const SensorData & data,
                       std::unique_ptr<cuvslam::Odometry> & cuvslam_handle,
                       std::unique_ptr<cuvslam::GroundConstraint> & ground_constraint_handle,
                       bool planar_constraints,
                       int multicam_mode,
                       CuVSLAMInputMode input_mode,
                       std::vector<uint8_t *> & gpu_left_image_data,
                       std::vector<uint8_t *> & gpu_right_image_data,
                       std::vector<uint8_t *> & gpu_depth_image_data,
                       std::vector<size_t> & gpu_left_image_sizes,
                       std::vector<size_t> & gpu_right_image_sizes,
                       std::vector<size_t> & gpu_depth_image_sizes,
                       cuvslam::Rig & cuvslam_rig,
                       cudaStream_t & cuda_stream)
{
    // cuVSLAM verbosity level (0=none, 1=errors, 2=warnings, 3=info)
    cuvslam::SetVerbosity(0);

    cuvslam_rig = cuvslam::Rig();
    if(input_mode == CuVSLAMInputMode::RGBD)
    {
        UASSERT(data.cameraModels().size() == 1);
        const CameraModel & cameraModel = data.cameraModels()[0];
        if(!cameraModel.isValidForProjection())
        {
            UERROR("Invalid RGB-D camera model for cuVSLAM initialization!");
            return false;
        }

        cuvslam_rig.cameras.resize(1);
        auto & cam = cuvslam_rig.cameras[0];
        cam.size = {cameraModel.imageWidth(), cameraModel.imageHeight()};
        cam.principal = {static_cast<float>(cameraModel.cx()), static_cast<float>(cameraModel.cy())};
        cam.focal = {static_cast<float>(cameraModel.fx()), static_cast<float>(cameraModel.fy())};
        cam.distortion.model = cuvslam::Distortion::Model::Pinhole;
        cam.distortion.parameters.clear();
        cam.rig_from_camera = TocuVSLAMPose(cameraModel.localTransform());
        cam.border_top = 0;
        cam.border_bottom = 0;
        cam.border_left = 0;
        cam.border_right = 0;
    }
    else
    {
        cuvslam_rig.cameras.resize(data.stereoCameraModels().size()*2);

        // Handle stereo cameras
        for(size_t i = 0; i < data.stereoCameraModels().size(); ++i)
        {
            const StereoCameraModel & stereoModel = data.stereoCameraModels()[i];
            if(!stereoModel.isValidForProjection())
            {
                UERROR("Invalid stereo camera model %d for cuVSLAM initialization!", static_cast<int>(i));
                return false;
            }
            const CameraModel & leftModel = stereoModel.left();
            const CameraModel & rightModel = stereoModel.right();

            auto & cam_left = cuvslam_rig.cameras[i*2];
            auto & cam_right = cuvslam_rig.cameras[i*2+1];

            // Left camera
            cam_left.size = {leftModel.imageWidth(), leftModel.imageHeight()};
            cam_left.principal = {static_cast<float>(leftModel.cx()), static_cast<float>(leftModel.cy())};
            cam_left.focal = {static_cast<float>(leftModel.fx()), static_cast<float>(leftModel.fy())};
            cam_left.distortion.model = cuvslam::Distortion::Model::Pinhole;
            cam_left.distortion.parameters.clear();
            cam_left.rig_from_camera = TocuVSLAMPose(stereoModel.localTransform());
            cam_left.border_top = 0;
            cam_left.border_bottom = 0;
            cam_left.border_left = 0;
            cam_left.border_right = 0;

            // Right camera
            cam_right.size = {rightModel.imageWidth(), rightModel.imageHeight()};
            cam_right.principal = {static_cast<float>(rightModel.cx()), static_cast<float>(rightModel.cy())};
            cam_right.focal = {static_cast<float>(rightModel.fx()), static_cast<float>(rightModel.fy())};
            cam_right.distortion.model = cuvslam::Distortion::Model::Pinhole;
            cam_right.distortion.parameters.clear();
            Transform baseline_transform(1, 0, 0, stereoModel.baseline(),
                                         0, 1, 0, 0,
                                         0, 0, 1, 0);
            cam_right.rig_from_camera = TocuVSLAMPose(stereoModel.localTransform() * baseline_transform);
            cam_right.rig_from_camera.rotation = cam_left.rig_from_camera.rotation;
            cam_right.border_top = 0;
            cam_right.border_bottom = 0;
            cam_right.border_left = 0;
            cam_right.border_right = 0;
        }
    }

    const cuvslam::Odometry::Config configuration = CreateConfiguration(data, multicam_mode, input_mode);

    // Create tracker
    UTimer create_timer; create_timer.start();

    try {
        cuvslam_handle.reset(new cuvslam::Odometry(cuvslam_rig, configuration));
    }
    catch(const std::exception & e) {
        UERROR("Failed to initialize CUVSLAM tracker: %s", e.what());
        return false;
    }

    // Initialize gpu image data vectors and sizes
    const size_t image_sets_count = input_mode == CuVSLAMInputMode::Stereo ? data.stereoCameraModels().size() : 1;
    gpu_left_image_data.resize(image_sets_count, nullptr);
    gpu_right_image_data.resize(input_mode == CuVSLAMInputMode::Stereo ? image_sets_count : 0, nullptr);
    gpu_depth_image_data.resize(input_mode == CuVSLAMInputMode::RGBD ? 1 : 0, nullptr);
    gpu_left_image_sizes.resize(image_sets_count, 0);
    gpu_right_image_sizes.resize(input_mode == CuVSLAMInputMode::Stereo ? image_sets_count : 0, 0);
    gpu_depth_image_sizes.resize(input_mode == CuVSLAMInputMode::RGBD ? 1 : 0, 0);

    // initialize ground constraints
    if (planar_constraints) 
    {
        cuvslam::Pose identity_cuvslam = TocuVSLAMPose(Transform::getIdentity());
        try {
            ground_constraint_handle.reset(new cuvslam::GroundConstraint(
                identity_cuvslam,
                identity_cuvslam,
                identity_cuvslam));
        }
        catch(const std::exception & e) {
            UERROR("Failed to initialize CUVSLAM ground constraint: %s", e.what());
            return false;
        }
    }

    return true;
}

/*
Implementation based on Isaac ROS VisualSlamNode::VisualSlamImpl::CreateConfiguration()
Source: isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp:379-422
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/19be8c781a55dee9cfbe9f097adca3986638feb1/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp#L379-L422    
*/
cuvslam::Odometry::Config CreateConfiguration(const SensorData & data, int multicam_mode, CuVSLAMInputMode input_mode)
{
    cuvslam::Odometry::Config configuration = cuvslam::Odometry::GetDefaultConfig();

    // Core Visual Odometry Settings 
    configuration.use_motion_model = true;                  // Enable motion model for better tracking
    configuration.use_denoising = false;                    // Disable denoising by default
    configuration.use_gpu = true;                           // Use GPU acceleration
    configuration.rectified_stereo_camera = input_mode == CuVSLAMInputMode::Stereo; // Stereo camera configuration
    configuration.enable_observations_export = true;        // Export observations for external reading
    configuration.enable_landmarks_export = true;           // Required for visualization/lost detection

    // Odometry configuration (Vision-only, no IMU)
    configuration.odometry_mode =
            input_mode == CuVSLAMInputMode::RGBD ?
            cuvslam::Odometry::OdometryMode::RGBD :
            cuvslam::Odometry::OdometryMode::Multicamera;
    switch(multicam_mode)
    {
    case 0:
        configuration.multicam_mode = cuvslam::Odometry::MulticameraMode::Moderate;
        break;
    case 1:
        configuration.multicam_mode = cuvslam::Odometry::MulticameraMode::Performance;
        break;
    case 2:
    default:
        configuration.multicam_mode = cuvslam::Odometry::MulticameraMode::Precision;
        break;
    }
    if(input_mode == CuVSLAMInputMode::RGBD)
    {
        configuration.rgbd_settings.depth_camera_id = 0;
        if(data.depthRaw().type() == CV_16UC1)
        {
            configuration.rgbd_settings.depth_scale_factor = 1000.0f;
        }
        else
        {
            configuration.rgbd_settings.depth_scale_factor = 1.0f;
        }
        configuration.rgbd_settings.enable_depth_stereo_tracking = false;
    }
    configuration.debug_imu_mode = false;

    // Use for getting debug images and logs
    // configuration.debug_dump_directory = "/home/...your desired directory...";
    
    return configuration;
}

// ============================================================================
// GPU Memory Management
// ============================================================================

bool allocateGpuMemory(size_t size, uint8_t ** gpu_ptr, size_t * current_size)
{
    if(*current_size != size) {
        // Reallocate GPU memory if size changed
        if(*gpu_ptr != nullptr) {
            cudaFree(*gpu_ptr);
        }
        *gpu_ptr = nullptr;
        cudaError_t cuda_err = cudaMalloc(gpu_ptr, size);
        if(cuda_err != cudaSuccess) {
            UERROR("Failed to allocate GPU memory: %s", cudaGetErrorString(cuda_err));
            return false;
        }
        *current_size = size;
    }
    return true;
}

bool copyToGpuAsync(const cv::Mat & cpu_image, uint8_t * gpu_ptr, size_t size, cudaStream_t & cuda_stream)
{
    // Initialize CUDA stream if not already done
    if(cuda_stream == nullptr) {
        cudaError_t stream_err = cudaStreamCreate(&cuda_stream);
        if(stream_err != cudaSuccess) {
            UERROR("Failed to create CUDA stream: %s", cudaGetErrorString(stream_err));
            return false;
        }
    }
    
    // Copy CPU data to GPU memory with async operation for better performance
    cudaError_t cuda_err = cudaMemcpyAsync(gpu_ptr, cpu_image.data, size, 
                                          cudaMemcpyHostToDevice, cuda_stream);
    if(cuda_err != cudaSuccess) {
        UERROR("Failed to copy image to GPU: %s", cudaGetErrorString(cuda_err));
        return false;
    }
    
    return true;
}

bool synchronizeGpuOperations(cudaStream_t & cuda_stream)
{
    if(cuda_stream) {
        cudaError_t cuda_err = cudaStreamSynchronize(cuda_stream);
        if(cuda_err != cudaSuccess) {
            UERROR("Failed to synchronize GPU operations: %s", cudaGetErrorString(cuda_err));
            return false;
        }
    }
    return true;
}

// ============================================================================
// Image Processing and Preparation
// ============================================================================

bool prepareImages(const SensorData & data, 
                   std::vector<cuvslam::Image> & cuvslam_images,
                   std::vector<cuvslam::Image> & cuvslam_depths,
                   CuVSLAMInputMode input_mode,
                   std::vector<uint8_t *> & gpu_left_image_data,
                   std::vector<uint8_t *> & gpu_right_image_data,
                   std::vector<uint8_t *> & gpu_depth_image_data,
                   std::vector<size_t> & gpu_left_image_sizes,
                   std::vector<size_t> & gpu_right_image_sizes,
                   std::vector<size_t> & gpu_depth_image_sizes,
                   cudaStream_t & cuda_stream)
{
    // Convert timestamp to nanoseconds (cuVSLAM expects nanoseconds)
    int64_t timestamp_ns = static_cast<int64_t>(data.stamp() * 1000000000.0);

    if(input_mode == CuVSLAMInputMode::RGBD)
    {
        UASSERT(data.cameraModels().size() == 1);
        UASSERT(gpu_left_image_data.size() == 1);
        UASSERT(gpu_depth_image_data.size() == 1);

        const CameraModel & camera_model = data.cameraModels()[0];
        cv::Mat rgb_image = data.imageRaw();
        cv::Mat depth_image = data.depthRaw();

        if(rgb_image.empty() || depth_image.empty()) {
            UERROR("No RGB or depth image available for cuVSLAM RGB-D odometry");
            return false;
        }
        if(rgb_image.channels() != 1 && rgb_image.channels() != 3) {
            UERROR("Unsupported RGB image format for cuVSLAM RGB-D odometry: %d channels", rgb_image.channels());
            return false;
        }
        if(depth_image.type() != CV_16UC1 && depth_image.type() != CV_32FC1) {
            UERROR("Unsupported depth image format for cuVSLAM RGB-D odometry: type=%d. Expected CV_16UC1 or CV_32FC1.", depth_image.type());
            return false;
        }
        if(rgb_image.cols != depth_image.cols || rgb_image.rows != depth_image.rows) {
            UERROR("RGB-D images must be aligned and have the same size. RGB=%dx%d depth=%dx%d",
                    rgb_image.cols,
                    rgb_image.rows,
                    depth_image.cols,
                    depth_image.rows);
            return false;
        }
        if(camera_model.imageWidth() > 0 && camera_model.imageHeight() > 0 &&
           (rgb_image.cols != camera_model.imageWidth() || rgb_image.rows != camera_model.imageHeight())) {
            UERROR("RGB image size (%dx%d) does not match camera model size (%dx%d)",
                    rgb_image.cols,
                    rgb_image.rows,
                    camera_model.imageWidth(),
                    camera_model.imageHeight());
            return false;
        }

        cv::Mat processed_rgb_image = rgb_image;
        cuvslam::ImageData::Encoding rgb_encoding;
        if(rgb_image.channels() == 1) {
            rgb_encoding = cuvslam::ImageData::Encoding::MONO;
        }
        else {
            cv::cvtColor(rgb_image, processed_rgb_image, cv::COLOR_BGR2RGB);
            rgb_encoding = cuvslam::ImageData::Encoding::RGB;
        }
        if(!processed_rgb_image.isContinuous()) {
            processed_rgb_image = processed_rgb_image.clone();
        }
        if(!depth_image.isContinuous()) {
            depth_image = depth_image.clone();
        }

        const size_t rgb_image_size = processed_rgb_image.total() * processed_rgb_image.elemSize();
        const size_t depth_image_size = depth_image.total() * depth_image.elemSize();

        if(!allocateGpuMemory(rgb_image_size, &gpu_left_image_data[0], &gpu_left_image_sizes[0])) {
            UERROR("PREPARE RGB-D IMAGES: Failed to allocate GPU memory for RGB image");
            return false;
        }
        if(!copyToGpuAsync(processed_rgb_image, gpu_left_image_data[0], rgb_image_size, cuda_stream)) {
            UERROR("PREPARE RGB-D IMAGES: Failed to copy RGB image to GPU");
            return false;
        }

        if(!allocateGpuMemory(depth_image_size, &gpu_depth_image_data[0], &gpu_depth_image_sizes[0])) {
            UERROR("PREPARE RGB-D IMAGES: Failed to allocate GPU memory for depth image");
            return false;
        }
        if(!copyToGpuAsync(depth_image, gpu_depth_image_data[0], depth_image_size, cuda_stream)) {
            UERROR("PREPARE RGB-D IMAGES: Failed to copy depth image to GPU");
            return false;
        }

        cuvslam::Image rgb_cuvslam_image;
        rgb_cuvslam_image.width = rgb_image.cols;
        rgb_cuvslam_image.height = rgb_image.rows;
        rgb_cuvslam_image.pixels = gpu_left_image_data[0];
        rgb_cuvslam_image.timestamp_ns = timestamp_ns;
        rgb_cuvslam_image.camera_index = 0;
        rgb_cuvslam_image.pitch = processed_rgb_image.step;
        rgb_cuvslam_image.encoding = rgb_encoding;
        rgb_cuvslam_image.data_type = cuvslam::ImageData::DataType::UINT8;
        rgb_cuvslam_image.is_gpu_mem = true;
        cuvslam_images.push_back(rgb_cuvslam_image);

        cuvslam::Image depth_cuvslam_image;
        depth_cuvslam_image.width = depth_image.cols;
        depth_cuvslam_image.height = depth_image.rows;
        depth_cuvslam_image.pixels = gpu_depth_image_data[0];
        depth_cuvslam_image.timestamp_ns = timestamp_ns;
        depth_cuvslam_image.camera_index = 0;
        depth_cuvslam_image.pitch = depth_image.step;
        depth_cuvslam_image.encoding = cuvslam::ImageData::Encoding::MONO;
        depth_cuvslam_image.data_type =
                depth_image.type() == CV_16UC1 ?
                cuvslam::ImageData::DataType::UINT16 :
                cuvslam::ImageData::DataType::FLOAT32;
        depth_cuvslam_image.is_gpu_mem = true;
        cuvslam_depths.push_back(depth_cuvslam_image);

        if(!synchronizeGpuOperations(cuda_stream)) {
            UERROR("PREPARE RGB-D IMAGES: Failed to synchronize GPU operations");
            return false;
        }

        return true;
    }

    // Horizontally stitched images received by RTAB-Map
    cv::Mat left_image = data.imageRaw();
    cv::Mat right_image = data.rightRaw();

    // Validate basic image properties
    if(left_image.empty() || right_image.empty()) {
        UERROR("No left or right image available for stereo camera");
        return false;
    }
    if(left_image.channels() != 1 && left_image.channels() != 3) {
        UERROR("Unsupported left image format: %d channels", left_image.channels());
        return false;
    }
    if(right_image.channels() != 1 && right_image.channels() != 3) {
        UERROR("Unsupported right image format: %d channels", right_image.channels());
        return false;
    }

    // Convert image format for cuVSLAM - mono8 or rgb8
    cv::Mat processed_left_image;
    cv::Mat processed_right_image;
    cuvslam::ImageData::Encoding left_encoding;
    cuvslam::ImageData::Encoding right_encoding;
    
    // process left image - copies image if BGR to RGB conversion is needed
    processed_left_image = left_image;
    if(left_image.channels() == 1) {
        left_encoding = cuvslam::ImageData::Encoding::MONO;
    } else if(left_image.channels() == 3) {
        // convert from BGR to RGB
        cv::cvtColor(left_image, processed_left_image, cv::COLOR_BGR2RGB);
        left_encoding = cuvslam::ImageData::Encoding::RGB;
    } else {
        UERROR("Unsupported left image format: %d channels", left_image.channels());
        return false;
    }

    // process right image - copies image if BGR to RGB conversion is needed
    processed_right_image = right_image;
    if(right_image.channels() == 1) {
        right_encoding = cuvslam::ImageData::Encoding::MONO;
    } else if(right_image.channels() == 3) {
        // convert from BGR to RGB
        cv::cvtColor(right_image, processed_right_image, cv::COLOR_BGR2RGB);
        right_encoding = cuvslam::ImageData::Encoding::RGB;
    } else {
        UERROR("Unsupported right image format: %d channels", right_image.channels());
        return false;
    }
       
    int camera_index = 0;
    int stereo_index = 0;
    for(const StereoCameraModel & model : data.stereoCameraModels()) {
        // slice out the image for the current stereo pair
        // Assumes all images have the same width and height
        int left_image_width = model.left().imageWidth();
        int right_image_width = model.right().imageWidth();
        int left_image_height = model.left().imageHeight();
        int right_image_height = model.right().imageHeight();

        // makes a copy for the sliced images
        cv::Mat left_image_slice = processed_left_image(cv::Rect(stereo_index * left_image_width, 0, left_image_width, left_image_height)).clone();
        cv::Mat right_image_slice = processed_right_image(cv::Rect(stereo_index * right_image_width, 0, right_image_width, right_image_height)).clone();
        
        size_t left_image_size = left_image_slice.total() * left_image_slice.elemSize();
        size_t right_image_size = right_image_slice.total() * right_image_slice.elemSize();

        // Allocate GPU memory for left camera
        if(!allocateGpuMemory(left_image_size, &gpu_left_image_data[stereo_index], &gpu_left_image_sizes[stereo_index])) {
            UERROR("PREPARE IMAGES: Failed to allocate GPU memory for left image");
            return false;
        }
        if(!copyToGpuAsync(left_image_slice, gpu_left_image_data[stereo_index], left_image_size, cuda_stream)) {
            UERROR("PREPARE IMAGES: Failed to copy left image to GPU");
            return false;
        }

        // Create CUVSLAM_Image for left camera with GPU memory
        cuvslam::Image left_cuvslam_image;
        left_cuvslam_image.width = left_image_width;
        left_cuvslam_image.height = left_image_height;
        left_cuvslam_image.pixels = gpu_left_image_data[stereo_index];  // GPU memory pointer
        left_cuvslam_image.timestamp_ns = timestamp_ns;
        left_cuvslam_image.camera_index = camera_index;
        left_cuvslam_image.pitch = left_image_slice.step;
        left_cuvslam_image.encoding = left_encoding;
        left_cuvslam_image.data_type = cuvslam::ImageData::DataType::UINT8;
        left_cuvslam_image.is_gpu_mem = true;
        
        cuvslam_images.push_back(left_cuvslam_image);

        camera_index++;

        // Allocate GPU memory for right camera
        if(!allocateGpuMemory(right_image_size, &gpu_right_image_data[stereo_index], &gpu_right_image_sizes[stereo_index])) {
            UERROR("PREPARE IMAGES: Failed to allocate GPU memory for right image");
            return false;
        }
        
        if(!copyToGpuAsync(right_image_slice, gpu_right_image_data[stereo_index], right_image_size, cuda_stream)) {
            UERROR("PREPARE IMAGES: Failed to copy right image to GPU");
            return false;
        }

        cuvslam::Image right_cuvslam_image;
        right_cuvslam_image.width = right_image_width;
        right_cuvslam_image.height = right_image_height;
        right_cuvslam_image.pixels = gpu_right_image_data[stereo_index];  // GPU memory pointer
        right_cuvslam_image.timestamp_ns = timestamp_ns;
        right_cuvslam_image.camera_index = camera_index;
        right_cuvslam_image.pitch = right_image_slice.step;
        right_cuvslam_image.encoding = right_encoding;
        right_cuvslam_image.data_type = cuvslam::ImageData::DataType::UINT8;
        right_cuvslam_image.is_gpu_mem = true;

        cuvslam_images.push_back(right_cuvslam_image);
        
        stereo_index++;
        camera_index++;
    }

    // Synchronize all async GPU operations before returning
    if(!synchronizeGpuOperations(cuda_stream)) {
        UERROR("PREPARE IMAGES: Failed to synchronize GPU operations");
        return false;
    }

    return true;
} 


#endif // RTABMAP_CUVSLAM

} // namespace rtabmap
