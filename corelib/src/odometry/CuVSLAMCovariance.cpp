/*
Copyright (c) 2026 Felix Toft

Internal cuVSLAM covariance conversion helpers.
*/

#include "CuVSLAMCovariance.h"

#include <eigen3/Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace rtabmap {

cv::Mat convertCuVSLAMCovariance(
        const cuvslam::PoseCovariance & cuvslam_covariance,
        const CuVSLAMCovarianceConfig & config,
        const cv::Mat & previous_covariance,
        bool * repaired)
{
    bool was_repaired = false;
    const int order[6] = {3, 4, 5, 0, 1, 2};
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    bool invalid_diagonal[6] = {false, false, false, false, false, false};

    for(int i = 0; i < 6; ++i)
    {
        for(int j = 0; j < 6; ++j)
        {
            const double value = static_cast<double>(cuvslam_covariance[order[i] * 6 + order[j]]);
            if(std::isfinite(value))
            {
                covariance(i, j) = value;
            }
            else
            {
                covariance(i, j) = 0.0;
                invalid_diagonal[i] = invalid_diagonal[i] || i == j;
                was_repaired = true;
            }
        }
    }

    was_repaired = was_repaired ||
            (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-9;
    covariance = 0.5 * (covariance + covariance.transpose()).eval();

    const double position_scale = config.use_raw ? 1.0 : config.position_scale;
    const double orientation_scale = config.use_raw ? 1.0 : config.orientation_scale;
    double axis_scale[6];
    for(int i = 0; i < 6; ++i)
    {
        axis_scale[i] = std::sqrt(i < 3 ? position_scale : orientation_scale);
    }
    for(int i = 0; i < 6; ++i)
    {
        for(int j = 0; j < 6; ++j)
        {
            covariance(i, j) *= axis_scale[i] * axis_scale[j];
        }
    }

    double target_diagonal[6];
    for(int i = 0; i < 6; ++i)
    {
        const double floor = i < 3 ? config.position_floor : config.orientation_floor;
        const double ceiling = i < 3 ? config.position_ceiling : config.orientation_ceiling;
        const double fallback = i < 3 ? config.fallback_position : config.fallback_orientation;
        double variance = covariance(i, i);
        if(!std::isfinite(variance) || variance <= 0.0 || invalid_diagonal[i])
        {
            variance = fallback;
            invalid_diagonal[i] = true;
            was_repaired = true;
        }
        const double bounded = std::max(floor, std::min(ceiling, variance));
        was_repaired = was_repaired || bounded != variance;
        target_diagonal[i] = bounded;
        covariance(i, i) = bounded;
    }
    for(int i = 0; i < 6; ++i)
    {
        if(invalid_diagonal[i])
        {
            for(int j = 0; j < 6; ++j)
            {
                if(i != j)
                {
                    covariance(i, j) = 0.0;
                    covariance(j, i) = 0.0;
                }
            }
        }
    }

    const double max_correlation = 0.999;
    for(int i = 0; i < 6; ++i)
    {
        for(int j = i + 1; j < 6; ++j)
        {
            const double limit = max_correlation * std::sqrt(target_diagonal[i] * target_diagonal[j]);
            double value = covariance(i, j);
            if(!std::isfinite(value))
            {
                value = 0.0;
                was_repaired = true;
            }
            const double bounded = std::max(-limit, std::min(limit, value));
            was_repaired = was_repaired || bounded != value;
            covariance(i, j) = bounded;
            covariance(j, i) = bounded;
        }
    }

    Eigen::Matrix<double, 6, 6> inv_standard_deviation = Eigen::Matrix<double, 6, 6>::Zero();
    for(int i = 0; i < 6; ++i)
    {
        inv_standard_deviation(i, i) = 1.0 / std::sqrt(target_diagonal[i]);
    }
    Eigen::Matrix<double, 6, 6> correlation =
            inv_standard_deviation * covariance * inv_standard_deviation;
    correlation = 0.5 * (correlation + correlation.transpose()).eval();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > solver(correlation);
    if(solver.info() == Eigen::Success)
    {
        Eigen::Matrix<double, 6, 1> eigenvalues = solver.eigenvalues();
        for(int i = 0; i < 6; ++i)
        {
            if(eigenvalues[i] < 1e-6)
            {
                eigenvalues[i] = 1e-6;
                was_repaired = true;
            }
        }
        correlation = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();

        Eigen::Matrix<double, 6, 6> normalization = Eigen::Matrix<double, 6, 6>::Zero();
        for(int i = 0; i < 6; ++i)
        {
            normalization(i, i) = 1.0 / std::sqrt(std::max(correlation(i, i), 1e-12));
        }
        correlation = normalization * correlation * normalization;
    }
    else
    {
        correlation.setIdentity();
        was_repaired = true;
    }

    Eigen::Matrix<double, 6, 6> standard_deviation = Eigen::Matrix<double, 6, 6>::Zero();
    for(int i = 0; i < 6; ++i)
    {
        standard_deviation(i, i) = std::sqrt(target_diagonal[i]);
    }
    covariance = standard_deviation * correlation * standard_deviation;

    if(!config.use_raw &&
       config.decrease_smoothing > 0.0 &&
       previous_covariance.rows == 6 &&
       previous_covariance.cols == 6 &&
       previous_covariance.type() == CV_64FC1)
    {
        Eigen::Matrix<double, 6, 6> smoothing_scale = Eigen::Matrix<double, 6, 6>::Identity();
        for(int i = 0; i < 6; ++i)
        {
            const double previous_variance = previous_covariance.at<double>(i, i);
            const double current_variance = covariance(i, i);
            if(std::isfinite(previous_variance) && previous_variance > current_variance)
            {
                const double smoothed =
                        config.decrease_smoothing * previous_variance +
                        (1.0 - config.decrease_smoothing) * current_variance;
                smoothing_scale(i, i) = std::sqrt(smoothed / current_variance);
            }
        }
        covariance = smoothing_scale * covariance * smoothing_scale;
    }

    cv::Mat result(6, 6, CV_64FC1);
    for(int i = 0; i < 6; ++i)
    {
        for(int j = 0; j < 6; ++j)
        {
            result.at<double>(i, j) = covariance(i, j);
        }
    }
    if(repaired)
    {
        *repaired = was_repaired;
    }
    return result;
}

} // namespace rtabmap
