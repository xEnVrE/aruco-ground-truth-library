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

    iCubCameraRelative(const std::string& robot_name, const std::string& laterality, const std::string& port_context, const std::string& fallback_context_name, const std::string& fallback_configuration_name, const bool& use_calibration = false, const std::string& calibration_path = "");

    iCubCameraRelative(const std::string& laterality, const std::string& data_path_left, const std::string& data_path_right, const std::size_t& width, const std::size_t& height, const double& fx_l, const double& cx_l, const double& fy_l, const double& cy_l, const double& fx_r, const double& cx_r, const double& fy_r, const double& cy_r, const bool& load_encoders_data, const bool& use_calibration = false, const std::string& calibration_path = "");

    ~iCubCameraRelative();

    bool step_frame() override;

    bool set_frame(const std::size_t& index) override;

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_pose(const bool& blocking) override;

private:
    const std::string log_name_ = "iCubCameraRelative";

    /* In case of offline mode, we need a second Camera to read the data of the second camera. */
    std::unique_ptr<iCubCamera> relative_camera_;
};

#endif /* ICUBCAMERARELATIVE_H */
