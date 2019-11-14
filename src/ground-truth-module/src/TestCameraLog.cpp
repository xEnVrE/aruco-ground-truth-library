/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/iCubCamera.h>

#include <Eigen/Dense>

#include <thread>

using namespace RobotsIO::Camera;


int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cout << "Synopsis: test-camera-log <camera_name> <laterality> <number_of_frames>"  << std::endl;
        std::cout << "          <camera_name> can be {iCubCamera}"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string camera_name = std::string(argv[1]);
    const std::string laterality = std::string(argv[2]);
    const std::size_t number_of_frames = std::stoi(argv[3]);

    std::unique_ptr<Camera> camera;
    if (camera_name == "iCubCamera")
    {
        camera = std::unique_ptr<iCubCamera>
        (
            new iCubCamera("icub", laterality, "test-camera-log", "", "")
        );
    }
    else
    {
        std::cout << "TestCamera::main. Error: unknown camera name " + camera_name + "." << std::endl;;
        return EXIT_FAILURE;
    }

    /* Enable log. */
    camera->start_log(".");
    std::cout << "Logger enabled." << std::endl;

    for (std::size_t i = 0; i < number_of_frames; i++)
    {
        std::cout << "Logging frame no. " << i << std::endl;

        camera->log_frame();

        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    /* Stop log. */
    camera->stop_log();
    std::cout << "Logger stopped." << std::endl;

    return EXIT_SUCCESS;
}
