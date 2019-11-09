/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBCAMERA_H
#define ICUBCAMERA_H

#include <Camera.h>

#include <Eigen/Dense>

#include <iCub/iKin/iKinFwd.h>

#include <opencv2/opencv.hpp>

#include <string>

#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>


class iCubCamera : public Camera
{
public:

    iCubCamera(const std::string& laterality, const std::string& port_context, const std::string& fallback_context_name, const std::string& fallback_configuration_name);

    iCubCamera(const std::string& data_path, const std::size_t& width, const double& height, const double& fx, const double& cx, const double& fy, const double& cy, const bool& load_encoders_data);

    ~iCubCamera();

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_pose(const bool& blocking) override;

    std::pair<bool, cv::Mat> get_rgb(const bool& blocking) override;

    std::pair<bool, Eigen::MatrixXf> get_depth(const bool& blocking) override;

    std::size_t get_auxiliary_data_size() override;

    std::pair<bool, Eigen::VectorXd> get_auxiliary_data(const bool& blocking) override;

protected:
    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_laterality_pose(const std::string& laterality, const bool& blocking);

    std::string get_laterality();

private:
    bool getLateralityEyePose(const std::string& laterality, yarp::sig::Vector& position, yarp::sig::Vector& orientation);

    yarp::os::Network yarp_;

    const std::string laterality_;

    bool use_driver_gaze_ = true;

    /* RGB-D sources. */

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_rgb_;

    /* Driver. */

    yarp::dev::PolyDriver driver_gaze_;

    yarp::dev::IGazeControl* gaze_control_;

    /* Fallback interface with encoders. */

    yarp::dev::PolyDriver drv_torso_;

    yarp::dev::IEncoders *itorso_;

    yarp::dev::PolyDriver drv_head_;

    yarp::dev::IEncoders *ihead_;

    iCub::iKin::iCubEye left_eye_kinematics_;

    iCub::iKin::iCubEye right_eye_kinematics_;

    /* Offline interface. */
    bool load_encoders_data_ = false;

    /* Log name to be used in messages printed by the class. */

    const std::string log_name_ = "iCubCamera";
};

#endif /* ICUBCAMERA_H */
