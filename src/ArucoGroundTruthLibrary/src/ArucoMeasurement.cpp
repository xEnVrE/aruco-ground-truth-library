/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoGroundTruthLibrary/ArucoMeasurement.h>

#include <RobotsIO/Camera/CameraParameters.h>

#include <iostream>

#include <opencv2/core/eigen.hpp>

using namespace ArucoGroundTruthLibrary;
using namespace Eigen;
using namespace RobotsIO::Camera;
using namespace bfl;
using namespace cv::aruco;


ArucoMeasurement::ArucoMeasurement(const int& dictionary, std::shared_ptr<Camera> camera) :
    camera_(std::move(camera))
{
    /* Setup dictionary. */
    dictionary_ = getPredefinedDictionary(dictionary);

    /* Get camera parameters. */
    bool valid_camera_parameters = false;
    CameraParameters parameters;
    std::tie(valid_camera_parameters, parameters) = camera_->parameters();
    if (!valid_camera_parameters)
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot get camera parameters."));

    /* Populate image size. */
    width_ = parameters.width;
    height_ = parameters.height;

    /* Populate intrinsic parameters. */
    cam_intrinsic_ = cv::Mat(3, 3, CV_64F, 0.0);
    cam_intrinsic_.at<double>(0, 0) = parameters.fx;
    cam_intrinsic_.at<double>(0, 2) = parameters.cx;
    cam_intrinsic_.at<double>(1, 1) = parameters.fy;
    cam_intrinsic_.at<double>(1, 2) = parameters.cy;
    cam_intrinsic_.at<double>(2, 2) = 1.0;

    /* Set zero distortion. */
    cam_distortion_ = cv::Mat(1, 4, CV_64F, 0.0);
    cam_distortion_.setTo(cv::Scalar(0.0));

    /* Log. */
    std::cout << log_name_ << "::ctor. Using a " << width_ << "x" << height_ << "camera." << std::endl;
    std::cout << log_name_ << "::ctor. Camera intrinsic parameters are " << cam_intrinsic_ << std::endl;
    std::cout << log_name_ << "::ctor. Camera distortion parameters are " << cam_distortion_ << std::endl;
}


ArucoMeasurement::~ArucoMeasurement()
{}


bool ArucoMeasurement::freeze(const Data& data)
{
    /* Reset pose validity. */
    valid_pose_ = false;

    /* Step frame in case of an offline camera. */
    if (!camera_->step_frame())
        return false;

    /* Freeze camera image. */
    bool valid_rgb = false;
    std::tie(valid_rgb, camera_rgb_image_) = camera_->rgb(true);
    if (!valid_rgb)
        return false;

    /* Freeze camera pose. */
    bool valid_pose = false;
    std::tie(valid_pose, camera_pose_) = camera_->pose(true);
    if (!valid_pose)
        return false;

    return true;
}


std::pair<bool, bfl::Data> ArucoMeasurement::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    throw(std::runtime_error(log_name_ + "::getOutputsize. Method not implemented."));
}


std::pair<bool, Data> ArucoMeasurement::innovation(const Data& predicted_measurements, const bfl::Data& measurements) const
{
    throw(std::runtime_error(log_name_ + "::getOutputsize. Method not implemented."));
}


std::pair<bool, bfl::Data> ArucoMeasurement::measure(const Data& data) const
{
    return std::make_pair(valid_pose_, std::make_pair(camera_pose_, pose_));
}


std::pair<std::size_t, std::size_t> ArucoMeasurement::getOutputSize() const
{
    throw(std::runtime_error(log_name_ + "::getOutputsize. Method not implemented."));
}


cv::Mat ArucoMeasurement::get_camera_intrinsic() const
{
    return cam_intrinsic_;
}


cv::Mat ArucoMeasurement::get_camera_distortion() const
{
    return cam_distortion_;
}


Eigen::Transform<double, 3, Eigen::Affine> ArucoMeasurement::get_frozen_camera_pose() const
{
    return camera_pose_;
}


cv::Mat ArucoMeasurement::get_frozen_rgb_image() const
{
    return camera_rgb_image_;
}


cv::Ptr<cv::aruco::Dictionary> ArucoMeasurement::get_dictionary()
{
    return dictionary_;
}


void ArucoMeasurement::set_pose(cv::Mat position, cv::Mat orientation)
{
    /* Store estimated pose. */
    Vector3d position_eigen;
    position_eigen(0) = position.at<double>(0, 0);
    position_eigen(1) = position.at<double>(1, 0);
    position_eigen(2) = position.at<double>(2, 0);

    Matrix3d orientation_eigen;
    cv::Mat orientation_matrix;
    cv::Rodrigues(orientation, orientation_matrix);
    cv::cv2eigen(orientation_matrix, orientation_eigen);

    pose_ = Translation<double, 3>(position_eigen);
    pose_.rotate(orientation_eigen);
    pose_w_camera_ = camera_pose_ * pose_;

    if(is_probe("pose"))
        get_probe("pose").set_data(pose_);

    if(is_probe("pose_w_camera"))
        get_probe("pose_w_camera").set_data(pose_w_camera_);
}


void ArucoMeasurement::set_pose(cv::Vec3d position, cv::Vec3d orientation)
{
    /* Store estimated pose. */
    Vector3d position_eigen;
    position_eigen(0) = position(0);
    position_eigen(1) = position(1);
    position_eigen(2) = position(2);

    Matrix3d orientation_eigen;
    cv::Mat orientation_matrix;
    cv::Rodrigues(orientation, orientation_matrix);
    cv::cv2eigen(orientation_matrix, orientation_eigen);

    pose_ = Translation<double, 3>(position_eigen);
    pose_.rotate(orientation_eigen);
    pose_w_camera_ = camera_pose_ * pose_;

    if(is_probe("pose"))
        get_probe("pose").set_data(pose_);

    if(is_probe("pose_w_camera"))
        get_probe("pose_w_camera").set_data(pose_w_camera_);
}
