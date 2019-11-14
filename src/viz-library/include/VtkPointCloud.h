/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef VTKPOINTCLOUD_H
#define VTKPOINTCLOUD_H

#include <VtkContent.h>

#include <Eigen/Dense>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/CameraParameters.h>

#include <vtkActor.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkVertexGlyphFilter.h>

#include <memory>
#include <string>


class VtkPointCloud : public VtkContent
{
public:
    VtkPointCloud(std::unique_ptr<RobotsIO::Camera::Camera> camera);

    virtual ~VtkPointCloud();

    void add_to_renderer(vtkRenderer& renderer) override;

    bool update(const bool& blocking) override;

private:
    void set_points(const Eigen::Ref<const Eigen::MatrixXd>& points);

    void set_colors(const Eigen::Ref<const Eigen::VectorXi>& valid_coordinates, const cv::Mat& rgb_image);

    vtkSmartPointer<vtkActor> actor_;

    vtkSmartPointer<vtkVertexGlyphFilter> glyph_filter_;

    vtkSmartPointer<vtkPolyDataMapper> mapper_;

    vtkSmartPointer<vtkPolyData> polydata_;

    vtkSmartPointer<vtkPoints> points_;

    vtkSmartPointer<vtkUnsignedCharArray> colors_;

    std::unique_ptr<RobotsIO::Camera::Camera> camera_;

    RobotsIO::Camera::CameraParameters camera_parameters_;

    std::vector<cv::Point> image_coordinates_;

    const std::string log_name_ = "VtkPointCloud";
};

#endif /* VTKPOINTCLOUD_H */
