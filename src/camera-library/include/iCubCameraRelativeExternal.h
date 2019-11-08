/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBCAMERARELATIVEEXTERNAL_H
#define ICUBCAMERARELATIVEEXTERNAL_H

#include <iCubCameraRelative.h>


class iCubCameraRelativeExternal : public iCubCameraRelative
{
public:

    iCubCameraRelativeExternal(const std::string& laterality, const std::string& port_prefix, const std::string& fallback_context_name, const std::string& fallback_configuration_name);

    ~iCubCameraRelativeExternal();

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_pose(const bool& blocking) override;

private:
    yarp::os::BufferedPort<yarp::sig::Vector> reference_pose_left_in_;

    yarp::os::BufferedPort<yarp::sig::Vector> reference_pose_right_in_;

    const std::string log_name_ = "iCubCameraRelativeExternal";
};

#endif /* ICUBCAMERARELATIVEEXTERNAL_H */
