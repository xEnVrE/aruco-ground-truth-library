/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ARUCOGROUNDTRUTHLIBRARY_TUNABLEREVERSELINKMEASUREMENT_H
#define ARUCOGROUNDTRUTHLIBRARY_TUNABLEREVERSELINKMEASUREMENT_H

#include <ArucoGroundTruthLibrary/ArucoMeasurement.h>
#include <ArucoGroundTruthLibrary/ReverseLinkMeasurement.h>

#include <Eigen/Dense>

#include <memory>
#include <string>

namespace ArucoGroundTruthLibrary{
    class TunableReverseLinkMeasurement;
}


class ArucoGroundTruthLibrary::TunableReverseLinkMeasurement : public ArucoGroundTruthLibrary::ReverseLinkMeasurement
{
public:
    TunableReverseLinkMeasurement(Eigen::Transform<double, 3, Eigen::Affine> transform_0, std::unique_ptr<ArucoGroundTruthLibrary::ArucoMeasurement> aruco_measurement);

    virtual ~TunableReverseLinkMeasurement();

    virtual void reset();

    virtual void shift_x(const double& length);

    virtual void shift_y(const double& length);

    virtual void shift_z(const double& length);

    virtual void rotate_x(const double& angle);

    virtual void rotate_y(const double& angle);

    virtual void rotate_z(const double& angle);

    virtual Eigen::Transform<double, 3, Eigen::Affine> current_transform() const;

protected:
    Eigen::Transform<double, 3, Eigen::Affine> evaluate_reverse_transform() override;

private:

    const Eigen::Transform<double, 3, Eigen::Affine> transform_0_;

    Eigen::Transform<double, 3, Eigen::Affine> current_transform_;

    const std::string log_name_ = "TunableReverseLinkMeasurement";
};

#endif /* ARUCOGROUNDTRUTHLIBRARY_REVERSELINKMEASUREMENT_H */
