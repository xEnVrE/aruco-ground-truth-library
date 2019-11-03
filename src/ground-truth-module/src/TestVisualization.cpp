/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <VtkContainer.h>
#include <VtkContent.h>
#include <VtkiCubHand.h>

int main()
{
    VtkContainer container(30, 600, 600);

    std::unique_ptr<VtkContent> hand = std::unique_ptr<VtkiCubHand>
    (
        new VtkiCubHand("test-visualization", "left")
    );
    container.add_content("hand", std::move(hand));
    container.run();

    return EXIT_SUCCESS;
}
