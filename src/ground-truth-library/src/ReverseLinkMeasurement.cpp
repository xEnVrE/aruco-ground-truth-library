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
{}


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
    std::pair<Transform<double, 3, Affine>, Transform<double, 3, Affine>> poses = any::any_cast<std::pair<Transform<double, 3, Affine>, Transform<double, 3, Affine>>>(pose_data);
    camera_pose_ = poses.first;

    /* Transform the pose. */
    pose_ = poses.second * reverse_transform_;
    pose_w_camera_ = camera_pose_ * pose_;

    /* Set data for probe. */
    if(is_probe("pose"))
        get_probe("pose").set_data(pose_);

    if(is_probe("pose_w_camera"))
        get_probe("pose_w_camera").set_data(pose_w_camera_);

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
    return std::make_pair(valid_pose_, std::make_pair(camera_pose_, pose_));
}


std::pair<std::size_t, std::size_t> ReverseLinkMeasurement::getOutputSize() const
{
    throw(std::runtime_error(log_name_ + "::getOutputsize. Method not implemented."));
}


void ReverseLinkMeasurement::initialize()
{
    reverse_transform_ = evaluate_reverse_transform();

    /* Log final. */
    std::cout << log_name_ << "::ctor. Reverse transform translation is " << reverse_transform_.translation() << std::endl;
    std::cout << log_name_ << "::ctor. Reverse transform rotation is " << reverse_transform_.rotation() << std::endl;
}
