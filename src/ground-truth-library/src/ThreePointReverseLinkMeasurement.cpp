/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ThreePointReverseLinkMeasurement.h>
#include "ReverseLinkMeasurement.h"

using namespace Eigen;


ThreePointReverseLinkMeasurement::ThreePointReverseLinkMeasurement(const Eigen::Vector3d& point_0, const Eigen::Vector3d& point_1, const Eigen::Vector3d& point_2, const Eigen::Vector3d& corner_offset, std::unique_ptr<ArucoMeasurement> aruco_measurement) :
    ReverseLinkMeasurement(std::move(aruco_measurement)),
    point_0_(point_0),
    point_1_(point_1),
    point_2_(point_2),
    corner_offset_(corner_offset)
{
    ReverseLinkMeasurement::initialize();
}


ThreePointReverseLinkMeasurement::~ThreePointReverseLinkMeasurement()
{}


Eigen::Transform<double, 3, Eigen::Affine> ThreePointReverseLinkMeasurement::evaluate_reverse_transform()
{
    /* Compose rotation matrix from link to marker slot. */
    Matrix3d rotation_link_to_marker;
    rotation_link_to_marker.col(0) = point_1_ - point_0_;
    rotation_link_to_marker.col(1) = point_2_ - point_0_;
    rotation_link_to_marker.col(2) = rotation_link_to_marker.col(0).cross(rotation_link_to_marker.col(1));
    rotation_link_to_marker.col(0).normalize();
    rotation_link_to_marker.col(1).normalize();
    rotation_link_to_marker.col(2).normalize();

    /* Verify that columns 0 and 1 are orthonormal. */
    std::cout << log_name_ << "::ctor. Dot product between x and y columns is " << rotation_link_to_marker.col(0).dot(rotation_link_to_marker.col(1)) << std::endl;

    /* Verify that RR^{T} is identity and that det(R) = + 1. */
    std::cout << log_name_ << "::ctor. RR^{T} is " << rotation_link_to_marker * rotation_link_to_marker.transpose() << std::endl;
    std::cout << log_name_ << "::ctor. det(R) is " << rotation_link_to_marker.determinant() << std::endl;

    /* Compose link to marker slot transformation.*/
    Transform<double, 3, Affine> link_to_marker_slot;
    link_to_marker_slot = Translation<double, 3>(point_0_);
    link_to_marker_slot.rotate(rotation_link_to_marker);

    /* Compose slot corner to marker/board offset transformation. */
    Transform<double, 3, Affine> slot_to_marker;
    slot_to_marker = Translation<double, 3>(corner_offset_);
    slot_to_marker.rotate(Matrix3d::Identity());

    /* Return inverse transformation required to move from aruco estimate to link estimate. */
    Transform<double, 3, Affine> link_to_marker = link_to_marker_slot * slot_to_marker;
    return link_to_marker.inverse();
}
