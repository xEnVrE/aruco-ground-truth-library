/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoGroundTruthLibrary/TunableReverseLinkMeasurement.h>
#include "ArucoGroundTruthLibrary/ReverseLinkMeasurement.h"

using namespace ArucoGroundTruthLibrary;
using namespace Eigen;


TunableReverseLinkMeasurement::TunableReverseLinkMeasurement(Eigen::Transform<double, 3, Eigen::Affine> transform_0, std::unique_ptr<ArucoMeasurement> aruco_measurement) :
    ReverseLinkMeasurement(std::move(aruco_measurement)),
    transform_0_(transform_0),
    current_transform_(transform_0)
{
    ReverseLinkMeasurement::initialize();
}


TunableReverseLinkMeasurement::~TunableReverseLinkMeasurement()
{}


void TunableReverseLinkMeasurement::reset()
{
    current_transform_ = transform_0_;

    ReverseLinkMeasurement::initialize();
}


void TunableReverseLinkMeasurement::shift_x(const double& length)
{
    current_transform_ *= Translation<double, 3>(length, 0.0, 0.0);

    ReverseLinkMeasurement::initialize();
}


void TunableReverseLinkMeasurement::shift_y(const double& length)
{
    current_transform_ *= Translation<double, 3>(0.0, length, 0.0);

    ReverseLinkMeasurement::initialize();
}


void TunableReverseLinkMeasurement::shift_z(const double& length)
{
    current_transform_ *= Translation<double, 3>(0.0, 0.0, length);

    ReverseLinkMeasurement::initialize();
}


void TunableReverseLinkMeasurement::rotate_x(const double& angle)
{
    current_transform_ *= AngleAxisd(angle, Vector3d::UnitX());

    ReverseLinkMeasurement::initialize();
}


void TunableReverseLinkMeasurement::rotate_y(const double& angle)
{
    current_transform_ *= AngleAxisd(angle, Vector3d::UnitY());

    ReverseLinkMeasurement::initialize();
}


void TunableReverseLinkMeasurement::rotate_z(const double& angle)
{
    current_transform_ *= AngleAxisd(angle, Vector3d::UnitZ());

    ReverseLinkMeasurement::initialize();
}


Eigen::Transform<double, 3, Eigen::Affine> TunableReverseLinkMeasurement::current_transform() const
{
    return current_transform_;
}


Eigen::Transform<double, 3, Eigen::Affine> TunableReverseLinkMeasurement::evaluate_reverse_transform()
{
    return current_transform_;
}
