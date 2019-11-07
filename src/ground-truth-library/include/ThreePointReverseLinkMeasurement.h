/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef THREEPOINTREVERSELINKMEASUREMENT_H
#define THREEPOINTREVERSELINKMEASUREMENT_H

#include <ReverseLinkMeasurement.h>

#include <ArucoMeasurement.h>

#include <Eigen/Dense>

#include <memory>


class ThreePointReverseLinkMeasurement : public ReverseLinkMeasurement
{
public:
    ThreePointReverseLinkMeasurement(const Eigen::Vector3d& point_0, const Eigen::Vector3d& point_1, const Eigen::Vector3d& point_2, const Eigen::Vector3d& corner_offset, std::unique_ptr<ArucoMeasurement> aruco_measurement);

    virtual ~ThreePointReverseLinkMeasurement();

protected:
    Eigen::Transform<double, 3, Eigen::Affine> evaluate_reverse_transform() override;

    const Eigen::Vector3d point_0_;

    const Eigen::Vector3d point_1_;

    const Eigen::Vector3d point_2_;

    const Eigen::Vector3d corner_offset_;

    const std::string log_name_ = "ThreePointReverseLinkMeasurement";
};

#endif /* THREEPOINTREVERSELINKMEASUREMENT_H */
