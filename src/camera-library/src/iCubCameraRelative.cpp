/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubCameraRelative.h>


iCubCameraRelative::iCubCameraRelative(const std::string& robot_name, const std::string& laterality, const std::string& port_context, const std::string& fallback_context_name, const std::string& fallback_configuration_name, const bool& use_calibration, const std::string& calibration_path) :
    iCubCamera(robot_name, laterality, port_context, fallback_context_name, fallback_configuration_name, use_calibration, calibration_path)
{}


iCubCameraRelative::iCubCameraRelative(const std::string& laterality, const std::string& data_path_left, const std::string& data_path_right, const std::size_t& width, const std::size_t& height, const double& fx_l, const double& cx_l, const double& fy_l, const double& cy_l, const double& fx_r, const double& cx_r, const double& fy_r, const double& cy_r, const bool& load_encoders_data, const bool& use_calibration, const std::string& calibration_path) :
    iCubCamera(data_path_right, "right", width, height, fx_r, cx_r, fy_r, cy_r, load_encoders_data, use_calibration, calibration_path)
{
    /* Initialize right camera. */
    relative_camera_= std::unique_ptr<iCubCamera>
    (
        new iCubCamera(data_path_left, "left", width, height, fx_l, cx_l, fy_l, cy_l, load_encoders_data)
    );
}


iCubCameraRelative::~iCubCameraRelative()
{}


bool iCubCameraRelative::step_frame()
{
    bool ok = iCubCamera::step_frame();

    if (is_offline())
        ok &= relative_camera_->step_frame();

    return ok;
}


bool iCubCameraRelative::set_frame(const std::size_t& index)
{
    bool ok = iCubCamera::set_frame(index);

    if (is_offline())
        ok &= relative_camera_->set_frame(index);

    return ok;
}


std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> iCubCameraRelative::get_pose(const bool& blocking)
{
    bool valid_left = false;
    Eigen::Transform<double, 3, Eigen::Affine> pose_left;
    std::tie(valid_left, pose_left) = relative_camera_->get_pose(false);
    if (!valid_left)
        return std::make_pair(false, Eigen::Transform<double, 3, Eigen::Affine>());

    bool valid_right = false;
    Eigen::Transform<double, 3, Eigen::Affine> pose_right;
    std::tie(valid_right, pose_right) = iCubCamera::get_pose(false);
    if (!valid_right)
        return std::make_pair(false, Eigen::Transform<double, 3, Eigen::Affine>());

    /* Evaluate the relative pose from left camera to right camera . */
    Eigen::Transform<double, 3, Eigen::Affine> pose_relative;
    pose_relative = pose_left.inverse() * pose_right;

    return std::make_pair(true, pose_relative);
}
