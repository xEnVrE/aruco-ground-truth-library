/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Camera.h>

#include <Eigen/src/Core/IO.h>

#include <cstdio>
#include <iostream>

using namespace Eigen;


Camera::Camera()
{}


Camera::~Camera()
{}


bool Camera::reset()
{
    if (is_offline())
        frame_index_ = -1;

    status_ = true;

    return true;
}


bool Camera::step_frame()
{
    if (is_offline())
    {
        frame_index_++;

        if ((frame_index_ + 1) > data_.cols())
        {
            status_ = false;

            return false;
        }
    }

    return true;
}


std::size_t Camera::get_frame() const
{
    if (is_offline())
        return frame_index_;

    return -1;
}


bool Camera::set_frame(const std::size_t& index)
{
    if (int(index) < 0)
        frame_index_ = -1;
    else
        frame_index_ = index;

    return true;
}


std::size_t Camera::get_auxiliary_data_size() const
{
    return 0;
}


std::pair<bool, VectorXd> Camera::get_auxiliary_data(const bool& blocking)
{
    return std::make_pair(false, VectorXd());
}


std::pair<bool, CameraParameters> Camera::get_parameters() const
{
    if (!parameters_.is_initialized())
        return std::make_pair(false, CameraParameters());

    return std::make_pair(true, parameters_);
}


std::pair<bool, MatrixXd> Camera::get_deprojection_matrix() const
{
    if (!deprojection_matrix_initialized_)
        return std::make_pair(false, MatrixXd());

    return std::make_pair(true, deprojection_matrix_);
}


bool Camera::get_status()
{
    return status_;
}


bool Camera::is_offline() const
{
    return offline_mode_;
}


bool Camera::enable_log(const std::string& path)
{
    log_path_ = path;
    if (log_path_.back() != '/')
        log_path_ += "/";

    log_.open(log_path_ + "data.txt");

    log_index_ = 0;

    return log_.is_open();
}


void Camera::stop_log()
{
    log_.close();
}


bool Camera::log_frame(const bool& log_depth)
{
    /* Get rgb image. */
    bool valid_rgb = false;
    cv::Mat rgb;
    std::tie(valid_rgb, rgb) = get_rgb(true);
    if (!valid_rgb)
        return false;

    /* TODO: complete implementation. */
    /* Get depth image. */
    bool valid_depth = false;
    MatrixXf depth;
    if (log_depth)
    {}

    /* Get camera pose .*/
    bool valid_pose = false;
    Transform<double, 3, Affine> pose;
    std::tie(valid_pose, pose) = get_pose(true);
    if (!valid_pose)
        return false;

    /* Get auxiliary data. */
    bool is_aux_data = false;
    VectorXd aux_data;
    std::tie(is_aux_data, aux_data) = get_auxiliary_data(true);

    /* Eigen precision format .*/
    IOFormat full_precision(FullPrecision);

    /* Save frame .*/
    AngleAxisd angle_axis(pose.rotation());
    VectorXd angle(1);
    angle(0) = angle_axis.angle();

    if (valid_rgb)
        cv::imwrite(log_path_ + "rgb_" + std::to_string(log_index_) + ".png", rgb);
    if (valid_depth)
        ;
    log_ << log_index_ << " "
         << pose.translation().transpose().format(full_precision) << " "
         << angle_axis.axis().transpose().format(full_precision) << " "
         << angle.format(full_precision);

    if (is_aux_data)
        log_ << " " << aux_data.transpose().format(full_precision);

    log_ << std::endl;

    log_index_++;

    return true;
}


Camera::Camera(const std::string& data_path, const std::size_t& width, const double& height, const double& fx, const double& cx, const double& fy, const double& cy) :
    data_path_(data_path),
    offline_mode_(true)
{
    /* Set intrinsic parameters. */
    parameters_.width = width;
    parameters_.height = height;
    parameters_.fx = fx;
    parameters_.cx = cx;
    parameters_.fy = fy;
    parameters_.cy = cy;
    parameters_.set_initialized();

    /* Fix data path. */
    if (data_path_.back() != '/')
        data_path_ += '/';
}


std::pair<bool, Transform<double, 3, Affine>> Camera::get_pose_offline()
{
    VectorXd data = data_.col(frame_index_);

    Vector3d position = data.segment<3>(1);
    VectorXd axis_angle = data.segment<4>(1 + 3);
    AngleAxisd angle_axis(axis_angle(3), axis_angle.head<3>());

    Transform<double, 3, Affine> pose;
    pose = Translation<double, 3>(position);
    pose.rotate(angle_axis);

    return std::make_pair(true, pose);
}


std::pair<bool, cv::Mat> Camera::get_rgb_offline()
{
    const std::string file_name = data_path_ + "rgb_" + std::to_string(frame_index_) + ".png";
    cv::Mat image = cv::imread(data_path_ + "rgb_" + std::to_string(frame_index_) + ".png", cv::IMREAD_COLOR);

    if (image.empty())
    {
        std::cout << log_name_ << "::get_rgb_offline. Warning: frame " << file_name << " is empty!" << std::endl;
        return std::make_pair(false, cv::Mat());
    }
    cv::resize(image, image, cv::Size(parameters_.width, parameters_.height));

    return std::make_pair(true, image);
}


std::pair<bool, MatrixXf> Camera::get_depth_offline()
{
    std::FILE* in;
    const std::string file_name = data_path_ + "depth_" + std::to_string(frame_index_) + ".float";

    if ((in = std::fopen(file_name.c_str(), "rb")) == nullptr)
    {
        std::cout << log_name_ << "::get_depth_offline. Error: cannot load depth frame " + file_name;
        return std::make_pair(true, MatrixXf());
    }

    /* Load image size .*/
    std::size_t dims[2];
    if (std::fread(dims, sizeof(dims), 1, in) != 1)
        return std::make_pair(false, MatrixXf());

    /* Load image. */
    float float_image_raw[dims[0] * dims[1]];
    if (std::fread(float_image_raw, sizeof(float), dims[0] * dims[1], in) != dims[0] * dims[1])
        return std::make_pair(false, MatrixXf());

    /* Store image. */
    MatrixXf float_image(dims[1], dims[0]);
    float_image = Map<Matrix<float, -1, -1, RowMajor>>(float_image_raw, dims[1], dims[0]);

    std::fclose(in);

    return std::make_pair(true, float_image);
}


std::pair<bool, VectorXd> Camera::get_auxiliary_data_offline()
{
    VectorXd data = data_.col(frame_index_);

    if (get_auxiliary_data_size() == 0)
        return std::make_pair(false, VectorXd());

    return std::make_pair(true, data.segment(8, get_auxiliary_data_size()));
}


std::pair<bool, MatrixXd> Camera::read_data_from_file()
{
    MatrixXd data;
    const std::string file_name = data_path_ + "data.txt";
    const std::size_t num_fields = 8 + get_auxiliary_data_size();

    std::ifstream istrm(file_name);
    if (!istrm.is_open())
    {
        std::cout << log_name_ + "::read_data_from_file. Error: failed to open " << file_name << std::endl;

        return std::make_pair(false, MatrixXd(0,0));
    }
    else
    {
        std::vector<std::string> istrm_strings;
        std::string line;
        while (std::getline(istrm, line))
        {
            istrm_strings.push_back(line);
        }

        data.resize(num_fields, istrm_strings.size());
        std::size_t found_lines = 0;
        for (auto line : istrm_strings)
        {
            std::size_t found_fields = 0;
            std::string number_str;
            std::istringstream iss(line);

            while (iss >> number_str)
            {
                std::size_t index = (num_fields * found_lines) + found_fields;
                *(data.data() + index) = std::stod(number_str);
                found_fields++;
            }
            if (num_fields != found_fields)
            {
                std::cout << log_name_ + "::read_data_from_file. Error: malformed input file " << file_name << std::endl;

                return std::make_pair(false, MatrixXd(0,0));
            }
            found_lines++;
        }

        istrm.close();

        return std::make_pair(true, data);
    }
}


bool Camera::initialize()
{
    bool ok = true;

    /* Cache the deprojection matrix once for all. */
    ok &= evaluate_deprojection_matrix();

    /* If offline mode, load data from file. */
    if (is_offline())
    {
        bool valid_data = false;
        std::tie(valid_data, data_) = read_data_from_file();
        if (!valid_data)
            throw(std::runtime_error(log_name_ + "::initialize. Cannot load offline data from " + data_path_));
    }

    return ok;
}


bool Camera::evaluate_deprojection_matrix()
{
    if (!parameters_.is_initialized())
        throw(std::runtime_error(log_name_ + "::reset. Camera parameters not initialized. Did you initialize the class member 'parameters_' in the derived class?."));

    /* Allocate storage. */
    deprojection_matrix_.resize(3, parameters_.width * parameters_.height);

    // Evaluate deprojection matrix
    int i = 0;
    for (std::size_t u = 0; u < parameters_.width; u++)
    {
        for (std::size_t v = 0; v < parameters_.height; v++)
        {
            deprojection_matrix_(0, i) = (u - parameters_.cx) / parameters_.fx;
            deprojection_matrix_(1, i) = (v - parameters_.cy) / parameters_.fy;
            deprojection_matrix_(2, i) = 1.0;

            i++;
        }
    }

    deprojection_matrix_initialized_ = true;

    return true;
}
