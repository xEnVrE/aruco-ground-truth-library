/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef VTKUPDATEHANDLER_H
#define VTKUPDATEHANDLER_H

#include <VtkContainer.h>

#include <vtkCommand.h>


class vtkUpdateHandler : public vtkCommand
{
public:
    static vtkUpdateHandler* New();

    void set_container(VtkContainer* container);

    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), void * vtkNotUsed(callData));

private:
    VtkContainer* container_;
};

#endif /* VTKUPDATEHANDLER_H */
