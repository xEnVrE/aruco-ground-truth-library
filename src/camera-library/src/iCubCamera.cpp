/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubCamera.h>

#include <iostream>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Image.h>

using namespace Eigen;
// using namespace iCub::iKin;
using namespace yarp::cv;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;


iCubCamera::iCubCamera(const std::string& laterality, const std::string& port_prefix, const std::string& fallback_context_name, const std::string& fallback_configuration_name) :
    laterality_(laterality)
{
    /* Check YARP network. */
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP network is not available."));
    }

    /* Check laterality. */
    if ((laterality_ != "left") && (laterality_ != "right"))
    {
        std::string err = log_name_ + "::ctor. Please use a valid laterality when constructing the iCubCamera instance.";
        throw(std::runtime_error(err));
    }

    /* Prepare properties for the GazeController. */
    Property properties;
    properties.put("device", "gazecontrollerclient");
    properties.put("remote", "/iKinGazeCtrl");
    properties.put("local", "/" + port_prefix + "/gazecontroller");

    /* Open driver. */
    bool ok = driver_gaze_.open(properties);
    if (!ok)
    {
        std::string err = log_name_ + "::ctor. Cannot open GazeController driver.";
        throw(std::runtime_error(err));
    }

    /* Try to retrieve the required view. */
    ok = driver_gaze_.view(gaze_control_) && (gaze_control_ != nullptr);
    if (!ok)
    {
        std::string err = log_name_ + "::ctor. Cannot open GazeController view.";
        throw(std::runtime_error(err));
    }

    //TODO: add configuration retrieval from a configuration file in case of driver failure

    /* Retrieve camera parameters. */
    Bottle info;
    std::string key;
    gaze_control_->getInfo(info);

    key = "camera_width_" + laterality_;
    if (info.find(key).isNull())
    {
        std::string err = log_name_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera width.";
        throw(std::runtime_error(err));
    }
    parameters_.width = info.find(key).asInt();

    key = "camera_height_" + laterality_;
    if (info.find(key).isNull())
    {
        std::string err = log_name_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera height.";
        throw(std::runtime_error(err));
    }
    parameters_.height = info.find(key).asInt();

    key = "camera_intrinsics_" + laterality_;
    if (info.find(key).isNull())
    {
        std::string err = log_name_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera intrinsic parameters.";
        throw(std::runtime_error(err));
    }
    Bottle *list = info.find(key).asList();
    parameters_.fx = list->get(0).asDouble();
    parameters_.cx = list->get(2).asDouble();
    parameters_.fy = list->get(5).asDouble();
    parameters_.cy = list->get(6).asDouble();

    parameters_.set_initialized();

    /* Log parameters. */
    std::cout << log_name_ + "::ctor. Camera parameters:" << std::endl;
    std::cout << log_name_ + "    - width: " << parameters_.width << std::endl;
    std::cout << log_name_ + "    - height: " << parameters_.height << std::endl;
    std::cout << log_name_ + "    - fx: " << parameters_.fx << std::endl;
    std::cout << log_name_ + "    - fy: " << parameters_.fy << std::endl;
    std::cout << log_name_ + "    - cx: " << parameters_.cx << std::endl;


    /* Open rgb input port. */
    if (!(port_rgb_.open("/" + port_prefix + "/rgbImage:i")))
    {
        std::string err = log_name_ + "::ctor. Error: cannot open rgb input port.";
        throw(std::runtime_error(err));
    }

    /* Open depth input port. */
    if (!(port_depth_.open("/" + port_prefix + "/depthImage:i")))
    {
        std::string err = log_name_ + "::ctor. Error: cannot open depth input port.";
        throw(std::runtime_error(err));
    }

    Camera::initialize();
}


iCubCamera::~iCubCamera()
{
    /* Close driver. */
    driver_gaze_.close();

    /* Close ports. */
    port_rgb_.close();

    port_depth_.close();
}


std::pair<bool, Transform<double, 3, Affine>> iCubCamera::get_pose(const bool& blocking)
{
    return get_laterality_pose(laterality_, blocking);
}


std::pair<bool, cv::Mat> iCubCamera::get_rgb(const bool& blocking)
{
    ImageOf<PixelRgb>* image_in;
    image_in = port_rgb_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, cv::Mat());

    cv::Mat image = yarp::cv::toCvMat(*image_in);

    return std::make_pair(true, image);
}


std::pair<bool, MatrixXf> iCubCamera::get_depth(const bool& blocking)
{
    // Get image
    ImageOf<PixelFloat>* image_in;
    image_in = port_depth_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, MatrixXf());

    cv::Mat image = yarp::cv::toCvMat(*image_in);
    Map<Eigen::Matrix<float, Dynamic, Dynamic, Eigen::RowMajor>> depth(image.ptr<float>(), image.rows, image.cols);

    return std::make_pair(true, depth);
}


bool iCubCamera::step_frame()
{
    return true;
}


std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> iCubCamera::get_laterality_pose(const std::string& laterality, const bool& blocking)
{
    Transform<double, 3, Affine> pose;

    yarp::sig::Vector position_yarp;
    yarp::sig::Vector orientation_yarp;

    bool ok = getLateralityEyePose(laterality, position_yarp, orientation_yarp);

    if (!ok)
        return std::make_pair(false, Transform<double, 3, Affine>());

    pose = Translation<double, 3>(toEigen(position_yarp));
    pose.rotate(AngleAxisd(orientation_yarp(3), toEigen(orientation_yarp).head<3>()));

    return std::make_pair(true, pose);
}


std::string iCubCamera::get_laterality()
{
    return laterality_;
}


bool iCubCamera::getLateralityEyePose(const std::string& laterality, yarp::sig::Vector& position, yarp::sig::Vector& orientation)
{
    if ((laterality != "left") && (laterality != "right"))
        return false;

    if (laterality == "left")
        return gaze_control_->getLeftEyePose(position, orientation);

    return gaze_control_->getRightEyePose(position, orientation);
}
