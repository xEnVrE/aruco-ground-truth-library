/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ReverseLinkMeasurement.h>

#include <BayesFilters/any.h>

#include <iostream>

using namespace Eigen;
using namespace bfl;


ReverseLinkMeasurement::ReverseLinkMeasurement(std::unique_ptr<ArucoMeasurement> aruco_measurement) :
    aruco_measurement_(std::move(aruco_measurement))
{
    /* TODO: take these parameters from a configuration file using libconfig. */
    Vector3d corner_origin;
    corner_origin(0) = -0.0303521;
    corner_origin(1) = 0.0243051;
    corner_origin(2) = 0.0389612;

    Vector3d corner_x;
    corner_x(0) = -0.0303521;
    corner_x(1) = -0.0256949;
    corner_x(2) = 0.0389612;

    Vector3d corner_y;
    corner_y(0) = 0.0196458;
    corner_y(1) = 0.0243051;
    corner_y(2) = 0.0384987;

    Vector3d corner_offsets;
    corner_offsets(0) = 0.0025;
    corner_offsets(1) = 0.0025;
    corner_offsets(2) = 0.0;

    /* Compose rotation matrix from link to marker slot. */
    Matrix3d rotation_link_to_marker;
    rotation_link_to_marker.col(0) = corner_x - corner_origin;
    rotation_link_to_marker.col(1) = corner_y - corner_origin;
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
    link_to_marker_slot = Translation<double, 3>(corner_origin);
    link_to_marker_slot.rotate(rotation_link_to_marker);

    /* Compose slot corner to marker/board offset transformation. */
    Transform<double, 3, Affine> slot_to_marker;
    slot_to_marker = Translation<double, 3>(corner_offsets);
    slot_to_marker.rotate(Matrix3d::Identity());

    /* Store inverse transformation required to move from aruco estimate to link estimate. */
    Transform<double, 3, Affine> link_to_marker = link_to_marker_slot * slot_to_marker;
    reverse_transform_ = link_to_marker.inverse();

    /* Log final . */
    std::cout << log_name_ << "::ctor. Reverse transform translation is " << reverse_transform_.translation() << std::endl;
    std::cout << log_name_ << "::ctor. Reverse transform rotation is " << reverse_transform_.rotation() << std::endl;
}


ReverseLinkMeasurement::~ReverseLinkMeasurement()
{}


bool ReverseLinkMeasurement::freeze(const Data& data)
{
    /* Reset pose validity. */
    valid_pose_ = false;

    /* Freeze underlying aruco measurement. */
    if (!aruco_measurement_->freeze(data))
        return false;

    /* Get the measure. */
    Data pose_data;
    std::tie(valid_pose_, pose_data) = aruco_measurement_->measure();
    pose_ = any::any_cast<Transform<double, 3, Affine>>(pose_data);

    /* Transform the pose. */
    pose_ = pose_ * reverse_transform_;

    /* Set data for probe. */
    if(is_probe("data_output"))
        get_probe("data_output").set_data(pose_);

    return valid_pose_;
}


std::pair<bool, bfl::Data> ReverseLinkMeasurement::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    throw(std::runtime_error(log_name_ + "::getOutputsize. Method not implemented."));
}


std::pair<bool, Data> ReverseLinkMeasurement::innovation(const Data& predicted_measurements, const bfl::Data& measurements) const
{
    throw(std::runtime_error(log_name_ + "::getOutputsize. Method not implemented."));
}


std::pair<bool, bfl::Data> ReverseLinkMeasurement::measure(const Data& data) const
{
    return std::make_pair(valid_pose_, pose_);
}


std::pair<std::size_t, std::size_t> ReverseLinkMeasurement::getOutputSize() const
{
    throw(std::runtime_error(log_name_ + "::getOutputsize. Method not implemented."));
}
