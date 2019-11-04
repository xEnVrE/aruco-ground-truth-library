/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubCamera.h>
#include <YarpCamera.h>

#include <VtkContainer.h>
#include <VtkContent.h>
#include <VtkPointCloud.h>
#include <VtkiCubHand.h>

#include <iostream>
#include <string>


int main(int argc, char** argv)
{
    if (argc != 7)
    {
        std::cout << "Synopsis: test-visualization <robot_name> <use_analogs> <hand_fk> <hand_aruco> <point_cloud> <camera>" << std::endl;
        return EXIT_FAILURE;
    }
    bool use_analogs = false;
    bool show_hand_fk = false;
    bool show_hand_aruco = false;
    bool show_point_cloud = false;

    std::string robot_name = std::string(argv[1]);
    if (std::string(argv[2]) == "true")
        use_analogs = true;
    if (std::string(argv[3]) == "true")
        show_hand_fk = true;
    if (std::string(argv[4]) == "true")
        show_hand_aruco = true;
    if (std::string(argv[5]) == "true")
        show_point_cloud = true;
    std::string camera_name = std::string(argv[6]);

    VtkContainer container(30, 600, 600);

    /* Show hand according to forward kinematics. */
    if (show_hand_fk)
    {
        std::unique_ptr<VtkContent> hand = std::unique_ptr<VtkiCubHand>
        (
            new VtkiCubHand(robot_name, "left", "test-visualization/hand_fk", use_analogs, {100.0 / 255.0, 160 / 255.0, 255.0 / 255.0})
        );
        container.add_content("hand_fk", std::move(hand));
    }

    /* Show hand according to aruco markers. */
    if (show_hand_aruco)
    {
        /* 'hand_left' is used to show the estimate of the left hand using the left camera. */
        std::unique_ptr<VtkContent> hand_left = std::unique_ptr<VtkiCubHand>
        (
            new VtkiCubHand(robot_name, "left", "test-visualization/hand_aruco_left", use_analogs, {220.0 / 255.0, 100 / 255.0, 100.0 / 255.0})
        );

	/* 'hand_right' is used to show the estimate of the left hand using the right camera. */
        std::unique_ptr<VtkContent> hand_right = std::unique_ptr<VtkiCubHand>
        (
            new VtkiCubHand(robot_name, "left", "test-visualization/hand_aruco_right", use_analogs, {220.0 / 255.0, 100 / 255.0, 200.0 / 255.0})
        );

        container.add_content("hand_aruco_left", std::move(hand_left));
        container.add_content("hand_aruco_right", std::move(hand_right));
    }

    /* Show point cloud. */
    if (show_point_cloud)
    {
        std::unique_ptr<Camera> camera;
        if (camera_name == "iCubCamera")
            camera = std::unique_ptr<iCubCamera>
            (
                new iCubCamera("right", "test-visualization", "", "")
            );
        else if (camera_name == "YarpCamera")
            camera = std::unique_ptr<YarpCamera>
            (
                new YarpCamera("test-visualization", 320, 240, 308.585174560547, 154.746704101562, 308.361328125, 117.926162719727)
            );
        else
        {
            std::cout << "Error: camera name not recognized." << std::endl;
            return EXIT_FAILURE;
        }

        std::unique_ptr<VtkPointCloud> pc = std::unique_ptr<VtkPointCloud>
        (
            new VtkPointCloud(std::move(camera))
        );
        container.add_content("point_cloud", std::move(pc));
    }

    container.run();

    return EXIT_SUCCESS;
}
