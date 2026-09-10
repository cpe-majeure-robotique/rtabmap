#include "odometry/CuVSLAMCovariance.h"

#include <eigen3/Eigen/Eigenvalues>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

void require(bool condition, const char * message)
{
    if(!condition)
    {
        std::cerr << "FAILED: " << message << std::endl;
        std::exit(1);
    }
}

rtabmap::CuVSLAMCovarianceConfig config(double smoothing = 0.0)
{
    return {true, 1.0, 1.0, 1e-6, 1e-6, 100.0, 100.0, 0.25, 0.1, smoothing};
}

Eigen::Matrix<double, 6, 6> toEigen(const cv::Mat & covariance)
{
    Eigen::Matrix<double, 6, 6> result;
    for(int i = 0; i < 6; ++i)
    {
        for(int j = 0; j < 6; ++j)
        {
            result(i, j) = covariance.at<double>(i, j);
        }
    }
    return result;
}

} // namespace

int main()
{
    // cuVSLAM is rotation-first, RTAB-Map is translation-first.
    cuvslam::PoseCovariance input = {};
    const float diagonal[6] = {4.0f, 5.0f, 6.0f, 1.0f, 2.0f, 3.0f};
    for(int i = 0; i < 6; ++i)
    {
        input[i * 6 + i] = diagonal[i];
    }

    bool repaired = true;
    cv::Mat output = rtabmap::convertCuVSLAMCovariance(input, config(), cv::Mat(), &repaired);
    const double expected[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    for(int i = 0; i < 6; ++i)
    {
        require(std::abs(output.at<double>(i, i) - expected[i]) < 1e-9, "axis reorder");
    }
    require(!repaired, "valid covariance marked as repaired");

    // Cross-correlations must survive the permutation.
    input[3 * 6 + 0] = 0.25f;
    input[0 * 6 + 3] = 0.25f;
    output = rtabmap::convertCuVSLAMCovariance(input, config());
    require(std::abs(output.at<double>(0, 3) - 0.25) < 1e-8, "cross-correlation reorder");

    // Invalid values must become conservative and positive definite.
    input[3 * 6 + 3] = -1.0f;
    input[4 * 6 + 4] = std::numeric_limits<float>::quiet_NaN();
    repaired = false;
    output = rtabmap::convertCuVSLAMCovariance(input, config(), cv::Mat(), &repaired);
    require(repaired, "invalid covariance not reported as repaired");
    require(std::abs(output.at<double>(0, 0) - 0.25) < 1e-9, "negative fallback");
    require(std::abs(output.at<double>(1, 1) - 0.25) < 1e-9, "NaN fallback");
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > solver(toEigen(output));
    require(solver.info() == Eigen::Success, "eigendecomposition");
    require(solver.eigenvalues().minCoeff() > 0.0, "positive-definite repair");

    // Degradation is immediate; only confidence recovery is smoothed.
    cuvslam::PoseCovariance small = {};
    cuvslam::PoseCovariance large = {};
    for(int i = 0; i < 6; ++i)
    {
        small[i * 6 + i] = 0.1f;
        large[i * 6 + i] = 2.0f;
    }
    cv::Mat previous = cv::Mat::eye(6, 6, CV_64FC1);
    rtabmap::CuVSLAMCovarianceConfig smooth_config = config(0.9);
    smooth_config.use_raw = false;
    output = rtabmap::convertCuVSLAMCovariance(small, smooth_config, previous);
    require(std::abs(output.at<double>(0, 0) - 0.91) < 1e-6, "decrease smoothing");
    output = rtabmap::convertCuVSLAMCovariance(large, smooth_config, previous);
    require(std::abs(output.at<double>(0, 0) - 2.0) < 1e-9, "delayed degradation");

    std::cout << "cuVSLAM covariance tests passed" << std::endl;
    return 0;
}
