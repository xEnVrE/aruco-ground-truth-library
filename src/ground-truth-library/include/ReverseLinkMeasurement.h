/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef REVERSELINKMEASUREMENT_H
#define REVERSELINKMEASUREMENT_H

#include <ArucoMeasurement.h>

#include <BayesFilters/Data.h>
#include <BayesFilters/MeasurementModel.h>

#include <Eigen/Dense>

#include <RobotsIO/Utils/ProbeContainer.h>

#include <memory>
#include <string>


class ReverseLinkMeasurement : public bfl::MeasurementModel,
                               public RobotsIO::Utils::ProbeContainer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ReverseLinkMeasurement(std::unique_ptr<ArucoMeasurement> aruco_measurement);

    virtual ~ReverseLinkMeasurement();

    bool freeze(const bfl::Data& data = bfl::Data()) override;

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const override;

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

protected:
    void initialize();

    virtual Eigen::Transform<double, 3, Eigen::Affine> evaluate_reverse_transform() = 0;

private:
    std::unique_ptr<ArucoMeasurement> aruco_measurement_;

    Eigen::Transform<double, 3, Eigen::Affine> pose_;

    Eigen::Transform<double, 3, Eigen::Affine> pose_w_camera_;

    Eigen::Transform<double, 3, Eigen::Affine> reverse_transform_;

    Eigen::Transform<double, 3, Eigen::Affine> camera_pose_;

    bool valid_pose_ = false;

    const std::string log_name_ = "ReverseLinkMeasurement";
};

#endif /* REVERSELINKMEASUREMENT_H */
