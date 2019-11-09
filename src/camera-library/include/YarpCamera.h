/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef YARPCAMERA_H
#define YARPCAMERA_H

#include <Camera.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <string>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>


class YarpCamera : public Camera
{
public:

    YarpCamera(const std::string& port_prefix, const std::size_t& width, const double& height, const double& fx, const double& cx, const double& fy, const double& cy);

    ~YarpCamera();

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_pose(const bool& blocking) override;

    std::pair<bool, cv::Mat> get_rgb(const bool& blocking) override;

    std::pair<bool, Eigen::MatrixXf> get_depth(const bool& blocking) override;

private:
    yarp::os::Network yarp_;

    /* RGB-D sources. */

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_rgb_;

    const std::string log_name_ = "YarpCamera";
};

#endif /* YARPCAMERA_H */
