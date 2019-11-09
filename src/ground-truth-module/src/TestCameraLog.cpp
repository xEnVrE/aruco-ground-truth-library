/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Camera.h>
#include <iCubCamera.h>

#include <Eigen/Dense>

#include <thread>


int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cout << "Synopsis: test-camera-log <camera_name> <laterality> <number_of_frames>"  << std::endl;
        std::cout << "          <camera_name> can be {iCubCamera}"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string camera_name = std::string(argv[2]);
    const std::string laterality = std::string(argv[3]);
    const std::size_t number_of_frames = std::stoi(argv[4]);

    std::unique_ptr<Camera> camera;
    if (camera_name == "iCubCamera")
    {
        camera = std::unique_ptr<iCubCamera>
        (
            new iCubCamera(laterality, "test-camera-log", "", "")
        );
    }
    else
    {
        std::cout << "TestCamera::main. Error: unknown camera name " + camera_name + "." << std::endl;;
        return EXIT_FAILURE;
    }

    /* Enable log. */
    camera->enable_log(".");

    for (std::size_t i = 0; i < number_of_frames; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));

        camera->log_frame();

        std::cout << "Logging frame no. " << i << std::endl;
    }

    /* Stop log. */
    camera->stop_log();

    return EXIT_SUCCESS;
}
