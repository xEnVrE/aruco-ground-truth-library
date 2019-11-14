/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/iCubCamera.h>
#include <RobotsIO/Camera/iCubCameraRelative.h>
#include <RobotsIO/Camera/YarpCamera.h>

#include <Eigen/Dense>

#include <thread>

using namespace RobotsIO::Camera;

int main(int argc, char** argv)
{
    if (argc != 5)
    {
        std::cout << "Synopsis: test-camera <robot_name> <camera_name> <laterality> <relative>"  << std::endl;
        std::cout << "          <camera_name> can be {iCubCamera}"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string robot_name = std::string(argv[1]);
    const std::string camera_name = std::string(argv[2]);
    const std::string laterality = std::string(argv[3]);
    const bool relative = (std::string(argv[4]) == "true");
    const bool external = (std::string(argv[5]) == "true");

    std::unique_ptr<Camera> camera;
    if (camera_name == "iCubCamera")
    {
        if (relative)
        {
            camera = std::unique_ptr<iCubCamera>
            (
                new iCubCameraRelative(robot_name, laterality, "test-camera", "", "")
            );
	}
        else
            camera = std::unique_ptr<iCubCamera>
            (
                new iCubCamera(robot_name, laterality, "test-camera", "", "")
            );
    }
    else
    {
        std::cout << "TestCamera::main. Error: unknown camera name " + camera_name + "." << std::endl;;
        return EXIT_FAILURE;
    }


    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        Eigen::Transform<double, 3, Eigen::Affine> pose;
        std::tie(std::ignore, pose) = camera->pose(true);

        std::cout << "TestCamera::main. Info: Camera position is" << std::endl;
        std::cout << pose.translation() << std::endl;

        Eigen::AngleAxisd angle_axis(pose.rotation());
        std::cout << "TestCamera::main. Info: Camera rotation (axis): " << angle_axis.axis() << std::endl;
        std::cout << "TestCamera::main. Info: Camera rotation (angle): " << angle_axis.angle() * 180.0 / M_PI << std::endl;
    }

    return EXIT_SUCCESS;
}
