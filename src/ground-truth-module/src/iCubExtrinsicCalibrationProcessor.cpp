/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoBoardMeasurement.h>
#include <iCubCamera.h>
// #include <YarpImageOfProbe.hpp>

#include <fstream>
#include <memory>

// #include <yarp/os/Network.h>


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Synopsis: icub-extr-calib-processor <data_path> <output>"  << std::endl;

        return EXIT_FAILURE;
    }
    std::string data_path = std::string(argv[1]);
    const std::string output_path = std::string(argv[2]);
    if (data_path.back() != '/')
        data_path += "/";

    /* Cameras. */
    std::shared_ptr<Camera> cam_left = std::make_shared<iCubCamera>(data_path + "left-camera", 640, 480, 468.672, 323.045, 467.73, 245.784, true);
    std::shared_ptr<Camera> cam_right = std::make_shared<iCubCamera>(data_path + "right-camera", 640, 480, 468.488, 301.274, 467.427, 245.503, true);

    /* Aruco measurements. */
    std::size_t n_x = 2;
    std::size_t n_y = 2;
    std::size_t dictionary_offset = 0;
    const double intra_marker = 0.005;
    const double marker_length = 0.02;
    ArucoBoardMeasurement aruco_left(cv::aruco::DICT_4X4_50, dictionary_offset, n_x, n_y, marker_length, intra_marker, cam_left);
    ArucoBoardMeasurement aruco_right(cv::aruco::DICT_4X4_50, dictionary_offset, n_x, n_y, marker_length, intra_marker, cam_right);

    // std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>> image_probe_left = std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>>
    // (
    //     new YarpImageOfProbe<yarp::sig::PixelRgb>("/icub-extr-calib-processor/aruco_left:o")
    // );
    // std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>> image_probe_right = std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>>
    // (
    //     new YarpImageOfProbe<yarp::sig::PixelRgb>("/icub-extr-calib-processor/aruco_right:o")
    // );
    // aruco_left.set_probe("image_output", std::move(image_probe_left));
    // aruco_right.set_probe("image_output", std::move(image_probe_right));
    // yarp::os::Network::connect("/icub-extr-calib-processor/aruco_left:o", "/aruco_left:i", "tcp");
    // yarp::os::Network::connect("/icub-extr-calib-processor/aruco_right:o", "/aruco_right:i", "tcp");

    /* Open file for output. */
    std::ofstream output;
    output.open(output_path);
    if (!output.is_open())
    {
        std::cout << "Cannot open output file " << output_path << "."  << std::endl;

        return EXIT_FAILURE;
    }

    /* Eigen precision format .*/
    Eigen::IOFormat full_precision(Eigen::FullPrecision);

    while (cam_left->get_status() && cam_right->get_status())
    {
        /* Estimate poses using AruCo. The Aruco measurement entity will also step the camera frame. */
        bool valid_frame = aruco_left.freeze();
        valid_frame &= aruco_right.freeze();
        if (valid_frame)
        {
            /* Get the pose of both cameras and marker in camera frames. */
            bfl::Data data_left;
            std::tie(std::ignore, data_left) = aruco_left.measure();
            std::pair<Eigen::Transform<double, 3, Eigen::Affine>, Eigen::Transform<double, 3, Eigen::Affine>> pair_left =
                bfl::any::any_cast<std::pair<Eigen::Transform<double, 3, Eigen::Affine>, Eigen::Transform<double, 3, Eigen::Affine>>>(data_left);

            bfl::Data data_right;
            std::tie(std::ignore, data_right) = aruco_right.measure();
            std::pair<Eigen::Transform<double, 3, Eigen::Affine>, Eigen::Transform<double, 3, Eigen::Affine>> pair_right =
                bfl::any::any_cast<std::pair<Eigen::Transform<double, 3, Eigen::Affine>, Eigen::Transform<double, 3, Eigen::Affine>>>(data_right);

            /* Evaluate extrinsic matrix according to camera poses. */
            Eigen::Transform<double, 3, Eigen::Affine> extrinsic = pair_left.first.inverse() * pair_right.first;

            /* Evaluate estimate of extrinsic matrix. */
            Eigen::Transform<double, 3, Eigen::Affine> extrinsic_estimate = pair_left.second * pair_right.second.inverse();

            /* Evaluate offset. */
            Eigen::Transform<double, 3, Eigen::Affine> extrinsic_offset = extrinsic.inverse() * extrinsic_estimate;

            /* Get input configuration (is the same for both cameras of iCub). */
            Eigen::VectorXd torso_head;
            Eigen::VectorXd eyes;
            std::tie(std::ignore, torso_head) = cam_left->get_auxiliary_data(false);
            eyes = torso_head.tail<3>() * M_PI / 180.0;

            /* Convert to axis angle. */
            Eigen::AngleAxisd extrinsic_rotation(extrinsic.rotation());
            Eigen::AngleAxisd extrinsic_estimate_rotation(extrinsic_estimate.rotation());
            Eigen::AngleAxisd extrinsic_offset_rotation(extrinsic_offset.rotation());
            Eigen::VectorXd extrinsic_angle(1);
            Eigen::VectorXd extrinsic_estimate_angle(1);
            Eigen::VectorXd extrinsic_offset_angle(1);
            extrinsic_angle(0) = extrinsic_rotation.angle();
            extrinsic_estimate_angle(0) = extrinsic_estimate_rotation.angle();
            extrinsic_offset_angle(0)= extrinsic_offset_rotation.angle();

            /* Store data. */
            output << eyes.transpose().format(full_precision) << " "
                   << extrinsic.translation().transpose().format(full_precision) << " "
                   << extrinsic_rotation.axis().transpose().format(full_precision) << " "
                   << extrinsic_angle.format(full_precision) << " "

                   << extrinsic_estimate.translation().transpose().format(full_precision) << " "
                   << extrinsic_estimate_rotation.axis().transpose().format(full_precision) << " "
                   << extrinsic_estimate_angle.format(full_precision) << " "

                   << extrinsic_offset.translation().transpose().format(full_precision) << " "
                   << extrinsic_offset_rotation.axis().transpose().format(full_precision) << " "
                   << extrinsic_offset_angle.format(full_precision) << " "
                   << std::endl;
        }
    }

    /* Close file. */
    output.close();

    return EXIT_SUCCESS;
}
