/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef VTKCONTENT_H
#define VTKCONTENT_H

#include <vtkRenderer.h>

class VtkContent
{
public:
    virtual ~VtkContent();

    virtual void add_to_renderer(vtkRenderer& renderer) = 0;

    virtual bool update(const bool& blocking) = 0;

private:
    const std::string log_name_ = "VtkContent";
};

#endif /* VTKCONTENT_H */
