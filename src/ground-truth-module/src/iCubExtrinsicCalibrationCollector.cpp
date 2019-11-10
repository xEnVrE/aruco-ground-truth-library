/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubCamera.h>

#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

#include <cstdlib>
#include <thread>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Synopsis: icub-extr-calib-collector <robot_name>"  << std::endl;

        return EXIT_FAILURE;
    }
    const std::string robot_name = std::string(argv[1]);


    Network yarp;
    if (!yarp.checkNetwork())
    {
        std::cout << "YARP not available." << std::endl;

        return EXIT_FAILURE;
    }

    const std::string port_prefix = "icub-extr-calib-collector";

    /* Head control. */
    Property properties;
    properties.put("device", "remote_controlboard");
    properties.put("local", "/" + port_prefix + "/head");
    properties.put("remote", "/" + robot_name + "/head");

    PolyDriver driver_head;
    IPositionControl* head_ctrl;
    if (!(driver_head.open(properties) && driver_head.view(head_ctrl) && (head_ctrl != nullptr)))
    {
        std::cout << "Cannot open driver for head control." << std::endl;

        return EXIT_FAILURE;
    }

    /* Cameras. */
    iCubCamera left_camera(robot_name, "left", "icub-extr-calib-collector/left-camera", "", "");
    iCubCamera right_camera(robot_name, "right", "icub-extr-calib-collector/right-camera", "", "");

    /* Connections for cameras. */
    std::string prefix = "/" + robot_name + "/cam";
    if (robot_name == "icub")
        prefix += "calib";
    std::string postfix = "";
    if (robot_name == "icub")
        postfix += "/out";
    if (!Network::connect(prefix + "/left" + postfix, "/icub-extr-calib-collector/left-camera/rgbImage:i", "tcp"))
    {
        std::cout << "Cannot connect camcalib output to left camera driver." << std::endl;

        return EXIT_FAILURE;
    }
    if (!Network::connect(prefix + "/right" + postfix, "/icub-extr-calib-collector/right-camera/rgbImage:i", "tcp"))
    {
        std::cout << "Cannot connect camcalib output to right camera driver." << std::endl;

        return EXIT_FAILURE;
    }

    /* Enable camera logging. */
    left_camera.enable_log("./left-camera");
    right_camera.enable_log("./right-camera");

    /* Define ranges for head joints (in degrees). */
    const double tilt_min = -12.0;
    const double tilt_max = 10.0;
    const double version_min = -15.0;
    const double version_max = 15.0;
    const double vergence_min = 0.0;
    const double vergence_max = 24.0;
    const double stride = 2.0;

    /* Joints to be moved .*/
    yarp::sig::VectorOf<int> joints(3);
    joints(0) = 3; /* Tilt. */
    joints(1) = 4; /* Version. */
    joints(2) = 5; /* Vergence. */

    /* Number of combinations .*/
    std::size_t total = 0;
    for (double tilt = tilt_min; tilt <= tilt_max; tilt += stride)
        for (double version = version_min; version <= version_max; version += stride)
            for (double vergence = vergence_min; vergence <= vergence_max; vergence += stride)
                total++;

    /* Collect data. */
    std::size_t counter = 0;
    for (double tilt = tilt_min; tilt <= tilt_max; tilt += stride)
        for (double version = version_min; version <= version_max; version += stride)
            for (double vergence = vergence_min; vergence <= vergence_max; vergence += stride)
            {
                /* Info. */
                std::cout << "Progress: " << counter <<  "/" << total << std::endl;

                /* Move camera. */
                yarp::sig::Vector references(3);
                references(0) = tilt;
                references(1) = version;
                references(2) = vergence;
                head_ctrl->positionMove(3, joints.data(), references.data());

                /* Wait before acquiring data. */
                std::this_thread::sleep_for(std::chrono::seconds(2));

                /* Acquire data. */
                left_camera.log_frame();
                right_camera.log_frame();

                counter++;
            }

    /* Save data. */
    left_camera.stop_log();
    right_camera.stop_log();

    /* Close head control driver. */
    driver_head.close();

    return EXIT_SUCCESS;
}
