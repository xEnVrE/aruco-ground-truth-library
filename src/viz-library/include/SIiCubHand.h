/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef SIICUBHAND_H
#define SIICUBHAND_H

#include <Camera.h>
#include <Eigen/Dense>

#include <iCubFingersEncoders.h>
#include <iCubForwardKinematics.h>

#include <SuperimposeMesh/SICAD.h>

#include <memory>

#include <opencv2/opencv.hpp>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>


class SIiCubHand
{
public:
    SIiCubHand(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const bool& use_analogs, const Camera& camera);

    virtual ~SIiCubHand();

    std::pair<bool, cv::Mat> render_image(const bool& blocking);

private:
    std::vector<double> transform_to_vector(const Eigen::Transform<double, 3, Eigen::Affine>& transform);

    yarp::os::Network yarp_;

    std::unique_ptr<SICAD> renderer_;

    SICAD::ModelStreamContainer meshes_;

    yarp::os::BufferedPort<yarp::sig::Vector> hand_pose_port_in_;

    std::unique_ptr<iCubForwardKinematics> forward_kinematics_;

    std::unique_ptr<iCubFingersEncoders> fingers_encoders_;

    const std::string log_name_ = "SIiCubHand";
};

#endif /* SIICUBHAND_H */
