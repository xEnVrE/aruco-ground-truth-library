/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubCameraRelativeExternal.h>

#include <yarp/eigen/Eigen.h>

using namespace Eigen;
using namespace yarp::eigen;


iCubCameraRelativeExternal::iCubCameraRelativeExternal(const std::string& laterality, const std::string& port_prefix, const std::string& fallback_context_name, const std::string& fallback_configuration_name) :
    iCubCameraRelative(laterality, port_prefix, fallback_context_name, fallback_configuration_name)
{
    if(!reference_pose_left_in_.open("/" + port_prefix + "/reference_left/pose:i"))
        throw(std::runtime_error(log_name_ + "::ctor. Cannot open left reference input port."));

    if(!reference_pose_right_in_.open("/" + port_prefix + "/reference_right/pose:i"))
        throw(std::runtime_error(log_name_ + "::ctor. Cannot open right reference input port."));
}


iCubCameraRelativeExternal::~iCubCameraRelativeExternal()
{
    reference_pose_left_in_.close();
    reference_pose_right_in_.close();
}


std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> iCubCameraRelativeExternal::get_pose(const bool& blocking)
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

    /* Evaluate the relative pose between the two cameras for reference purposes. */
    Eigen::Transform<double, 3, Eigen::Affine> pose_relative_fk;
    if (get_laterality() == "left")
        pose_relative_fk = pose_right.inverse() * pose_left;
    else
        pose_relative_fk = pose_left.inverse() * pose_right;

    /* Evaluate the relative pose using external references. */
    yarp::sig::Vector* reference_left_in = reference_pose_left_in_.read(blocking);
    if (reference_left_in == nullptr)
        return std::make_pair(false, Eigen::Transform<double, 3, Eigen::Affine>());

    yarp::sig::Vector* reference_right_in = reference_pose_right_in_.read(blocking);
    if (reference_right_in == nullptr)
        return std::make_pair(false, Eigen::Transform<double, 3, Eigen::Affine>());

    Eigen::Transform<double, 3, Eigen::Affine> reference_left;
    reference_left = Translation<double, 3>(toEigen(*reference_left_in).head<3>());
    AngleAxisd rotation_left(toEigen(*reference_left_in)(6), toEigen(*reference_left_in).segment<3>(3));
    reference_left.rotate(rotation_left);

    Eigen::Transform<double, 3, Eigen::Affine> reference_right;
    reference_right = Translation<double, 3>(toEigen(*reference_right_in).head<3>());
    AngleAxisd rotation_right(toEigen(*reference_right_in)(6), toEigen(*reference_right_in).segment<3>(3));
    reference_right.rotate(rotation_right);

    Eigen::Transform<double, 3, Eigen::Affine> pose_relative;
    if (get_laterality() == "left")
        pose_relative = reference_right * reference_left.inverse();
    else
        pose_relative = reference_left * reference_right.inverse();

    return std::make_pair(true, pose_relative);
}
