/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <VtkContainer.h>
#include <VtkContent.h>
#include <VtkiCubHand.h>

#include <cstdlib>

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Synopsis: test-visualization <robot_name> <use_analogs>" << std::endl;
        return EXIT_FAILURE;
    }
    std::string robot_name = std::string(argv[1]);
    bool use_analogs = false;
    if (std::string(argv[2]) == "true")
        use_analogs = true;


    VtkContainer container(30, 600, 600);

    std::unique_ptr<VtkContent> hand = std::unique_ptr<VtkiCubHand>
    (
        new VtkiCubHand(robot_name, "left", "test-visualization", use_analogs)
    );
    container.add_content("hand", std::move(hand));
    container.run();

    return EXIT_SUCCESS;
}
