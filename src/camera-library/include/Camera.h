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

#include <fstream>
#include <string>


class Camera
{
public:
    Camera();

    virtual ~Camera();

    virtual bool reset();

    virtual bool step_frame();

    virtual std::size_t get_frame() const;

    virtual bool set_frame(const std::size_t& index);

    virtual std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_pose(const bool& blocking) = 0;

    virtual std::pair<bool, cv::Mat> get_rgb(const bool& blocking) = 0;

    virtual std::pair<bool, Eigen::MatrixXf> get_depth(const bool& blocking) = 0;

    virtual std::size_t get_auxiliary_data_size() const;

    virtual std::pair<bool, Eigen::VectorXd> get_auxiliary_data(const bool& blocking);

    virtual std::pair<bool, CameraParameters> get_parameters() const;

    virtual std::pair<bool, Eigen::MatrixXd> get_deprojection_matrix() const;

    virtual bool get_status();

    virtual bool is_offline() const;

    virtual bool enable_log(const std::string& path);

    virtual void stop_log();

    virtual bool log_frame(const bool& log_depth = false);

protected:
    Camera(const std::string& data_path, const std::size_t& width, const double& height, const double& fx, const double& cx, const double& fy, const double& cy);

    virtual std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_pose_offline();

    virtual std::pair<bool, cv::Mat> get_rgb_offline();

    virtual std::pair<bool, Eigen::MatrixXf> get_depth_offline();

    virtual std::pair<bool, Eigen::VectorXd> get_auxiliary_data_offline();

    virtual std::pair<bool, Eigen::MatrixXd> read_data_from_file();

    virtual bool initialize();

    virtual bool evaluate_deprojection_matrix();

    CameraParameters parameters_;

    Eigen::MatrixXd deprojection_matrix_;

    bool deprojection_matrix_initialized_ = false;

    bool status_ = true;

    /* Data logging on file. */

    std::ofstream log_;

    std::string log_path_;

    std::size_t log_index_ = 0;

    /* Offline interface. */

    const bool offline_mode_ = false;

    std::string data_path_;

    std::ifstream data_in_;

    Eigen::MatrixXd data_;

    int frame_index_ = -1;

    /* Log name to be used in messages printed by the class. */

    const std::string log_name_ = "Camera";
};

#endif /* CAMERA_H */
