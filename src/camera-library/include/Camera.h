/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <CameraParameters.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <string>

class Camera
{
public:
    virtual ~Camera();

    virtual bool reset();

    virtual bool step_frame();

    virtual std::size_t get_frame() const;

    virtual bool set_frame(const std::size_t& index);

    virtual std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_pose(const bool& blocking) = 0;

    virtual std::pair<bool, cv::Mat> get_rgb(const bool& blocking) = 0;

    virtual std::pair<bool, Eigen::MatrixXf> get_depth(const bool& blocking) = 0;

    virtual std::pair<bool, CameraParameters> get_parameters() const;

    virtual std::pair<bool, Eigen::MatrixXd> get_deprojection_matrix() const;

protected:
    virtual bool initialize();

    virtual bool evaluate_deprojection_matrix();

    CameraParameters parameters_;

    Eigen::MatrixXd deprojection_matrix_;

    bool deprojection_matrix_initialized_ = false;

    const std::string log_name_ = "Camera";
};

#endif /* CAMERA_H */
