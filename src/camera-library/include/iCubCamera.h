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

// #include <iCub/iKin/iKinFwd.h>

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

    ~iCubCamera();

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_pose(const bool& blocking) override;

    std::pair<bool, cv::Mat> get_rgb(const bool& blocking) override;

    std::pair<bool, Eigen::MatrixXf> get_depth(const bool& blocking) override;

private:
    yarp::os::Network yarp_;

    const std::string laterality_;

    bool use_driver_gaze_ = true;

    /* RGB-D sources. */

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_rgb_;

    /* Driver. */

    yarp::dev::PolyDriver driver_gaze_;

    yarp::dev::IGazeControl* gaze_control_;

    /* Fallback interface with ports. */

    // yarp::os::BufferedPort<yarp::os::Bottle> port_encoder_head_;

    // yarp::os::BufferedPort<yarp::os::Bottle> port_encoder_torso_;

    // iCub::iKin::iCubEye icub_eye_kinematics_;

    const std::string log_name_ = "iCubCamera";
};

#endif /* ICUBCAMERA_H */
