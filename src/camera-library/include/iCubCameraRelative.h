/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBCAMERARELATIVE_H
#define ICUBCAMERARELATIVE_H

#include <iCubCamera.h>


class iCubCameraRelative : public iCubCamera
{
public:

    iCubCameraRelative(const std::string& laterality, const std::string& port_context, const std::string& fallback_context_name, const std::string& fallback_configuration_name);

    ~iCubCameraRelative();

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_pose(const bool& blocking) override;

private:
    const std::string log_name_ = "iCubCameraRelative";
};

#endif /* ICUBCAMERARELATIVE_H */
