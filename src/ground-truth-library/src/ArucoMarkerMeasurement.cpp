/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoMarkerMeasurement.h>

using namespace bfl;
using namespace cv::aruco;


ArucoMarkerMeasurement::ArucoMarkerMeasurement(const int& dictionary, const double& marker_length, std::shared_ptr<Camera> camera) :
    ArucoMeasurement(dictionary, std::move(camera)),
    marker_length_(marker_length)
{}


ArucoMarkerMeasurement::~ArucoMarkerMeasurement()
{}


bool ArucoMarkerMeasurement::freeze(const bfl::Data& data)
{
    /* Freeze measurements. */
    if(!ArucoMeasurement::freeze(data))
        return false;

    /* Get image .*/
    cv::Mat image = get_frozen_rgb_image();

    /* Perform markers detection. */
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> inliers;
    detectMarkers(image, get_dictionary(), inliers, ids);

    if (ids.size() > 0)
    {
        /* Perform marker pose estimation. */
        std::vector<cv::Vec3d> position;
        std::vector<cv::Vec3d> orientation;
        estimatePoseSingleMarkers(inliers, marker_length_, get_camera_intrinsic(), get_camera_distortion(), orientation, position);

        /* Generate image for probe if setup. */
        if(is_probe("image_output"))
        {
            probe_image_ = image.clone();
            drawDetectedMarkers(probe_image_, inliers, ids);
            for(int i = 0; i < ids.size(); i++)
                cv::aruco::drawAxis(probe_image_, get_camera_intrinsic(), get_camera_distortion(), orientation.at(i), position.at(i), 0.1);

            get_probe("image_output").set_data(probe_image_);
        }

        if ((position.size() != 0) && (orientation.size() != 0))
        {
            set_pose(position.at(0), orientation.at(0));
            return true;
        }

        return false;
    }

    return false;
}
