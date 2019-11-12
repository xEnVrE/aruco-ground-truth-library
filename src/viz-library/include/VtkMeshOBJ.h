
/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef VTKMESHOBJ_H
#define VTKMESHOBJ_H

#include <Eigen/Dense>

#include <vtkActor.h>
#include <vtkOBJResource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>

#include <memory>


class VtkMeshOBJ
{
public:
    VtkMeshOBJ(const std::string& mesh_path, const bool& use_mesh_resources, const std::tuple<double, double, double>& color, const double& opacity);

    virtual ~VtkMeshOBJ();

    void add_to_renderer(vtkRenderer& renderer);

    void set_pose(const Eigen::Transform<double, 3, Eigen::Affine>& pose);

private:
    vtkSmartPointer<vtkOBJResource> reader_;

    vtkSmartPointer<vtkPolyDataMapper> mapper_;

    vtkSmartPointer<vtkActor> mesh_actor_;
};

#endif /* VTKMESHOBJ_H */
