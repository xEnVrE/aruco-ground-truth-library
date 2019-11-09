/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Camera.h>
#include <iCubCamera.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Synopsis: test-camera-log-read <camera_name> <path>"  << std::endl;
        std::cout << "          <camera_name> can be {iCubCamera}"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string camera_name = std::string(argv[1]);
    const std::string data_path = std::string(argv[2]);

    std::unique_ptr<Camera> camera;
    if (camera_name == "iCubCamera")
    {
        camera = std::unique_ptr<iCubCamera>
        (
            new iCubCamera(data_path, 640.0, 480.0, 468.672, 323.045, 467.73, 245.784, true)
        );
    }
    else
    {
        std::cout << "TestCamera::main. Error: unknown camera name " + camera_name + "." << std::endl;;
        return EXIT_FAILURE;
    }

    /* Read data. */
    do
    {
        bool valid_rgb = false;
        cv::Mat rgb;
        std::tie(valid_rgb, rgb) = camera->get_rgb(false);
        if (!valid_rgb)
        {
            std::cout << "Cannot get camera rgb image for frame " << camera->get_frame() << std::endl;
            return EXIT_FAILURE;
        }

        bool valid_pose = false;
        Eigen::Transform<double, 3, Eigen::Affine> pose;
        std::tie(valid_pose, pose) = camera->get_pose(false);
        if (!valid_pose)
        {
            std::cout << "Cannot get camera pose for frame " << camera->get_frame() << std::endl;
            return EXIT_FAILURE;
        }

        bool valid_aux_data = false;
        Eigen::VectorXd aux_data;
        std::tie(valid_aux_data, aux_data) = camera->get_auxiliary_data(false);
        if (!valid_aux_data)
        {
            std::cout << "Cannot retrieve auxiliary data." << std::endl;
            return EXIT_FAILURE;
        }

        if (aux_data.size() != camera->get_auxiliary_data_size())
        {
            std::cout << "Auxiliary data size is different from expected. Expected " << camera->get_auxiliary_data_size()
                      << ", obtained " << aux_data.size() << std::endl;
            return EXIT_FAILURE;
        }

        Eigen::AngleAxisd angle_axis(pose.rotation());
        std::cout << "Frame: " << camera->get_frame() << std::endl;
        std::cout << "Camera pose:" << std::endl << pose.translation().transpose() << " " << angle_axis.axis().transpose() << " " << angle_axis.angle() << std::endl;
        std::cout << "Auxiliary data" << aux_data.transpose() << std::endl << std::endl;

    } while (camera->step_frame());

    return EXIT_SUCCESS;
}
