/*
Copyright (c) 2026 Felix Toft

Internal cuVSLAM covariance conversion helpers. This header is intentionally
kept in corelib/src: it is shared by OdometryCuVSLAM and focused numerical
tests, but is not part of RTAB-Map's public API.
*/

#ifndef RTABMAP_CUVSLAM_COVARIANCE_H_
#define RTABMAP_CUVSLAM_COVARIANCE_H_

#include <opencv2/core/core.hpp>
#include <cuvslam.h>

namespace rtabmap {

struct CuVSLAMCovarianceConfig
{
    bool use_raw;
    double position_scale;
    double orientation_scale;
    double position_floor;
    double orientation_floor;
    double position_ceiling;
    double orientation_ceiling;
    double fallback_position;
    double fallback_orientation;
    double decrease_smoothing;
};

cv::Mat convertCuVSLAMCovariance(
        const cuvslam::PoseCovariance & cuvslam_covariance,
        const CuVSLAMCovarianceConfig & config,
        const cv::Mat & previous_covariance = cv::Mat(),
        bool * repaired = 0);

} // namespace rtabmap

#endif // RTABMAP_CUVSLAM_COVARIANCE_H_
