/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubCameraRelative.h>


iCubCameraRelative::iCubCameraRelative(const std::string& laterality, const std::string& port_context, const std::string& fallback_context_name, const std::string& fallback_configuration_name) :
    iCubCamera(laterality, port_context, fallback_context_name, fallback_configuration_name)
{}


iCubCameraRelative::~iCubCameraRelative()
{}


std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> iCubCameraRelative::get_pose(const bool& blocking)
{
    bool valid_left = false;
    Eigen::Transform<double, 3, Eigen::Affine> pose_left;
    std::tie(valid_left, pose_left) = get_laterality_pose("left", blocking);
    if (!valid_left)
        return std::make_pair(false, Eigen::Transform<double, 3, Eigen::Affine>());

    bool valid_right = false;
    Eigen::Transform<double, 3, Eigen::Affine> pose_right;
    std::tie(valid_right, pose_right) = get_laterality_pose("right", blocking);
    if (!valid_right)
        return std::make_pair(false, Eigen::Transform<double, 3, Eigen::Affine>());

    /* Evaluate the relative pose between the two cameras. */
    Eigen::Transform<double, 3, Eigen::Affine> pose_relative;
    if (get_laterality() == "left")
        pose_relative = pose_left.inverse() * pose_right;
    else
        pose_relative = pose_right.inverse() * pose_left;

    return std::make_pair(true, pose_relative);
}
