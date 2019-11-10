/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoBoardMeasurement.h>

#include <Eigen/Dense>

#include <opencv2/aruco.hpp>

using namespace Eigen;
using namespace bfl;
using namespace cv::aruco;


ArucoBoardMeasurement::ArucoBoardMeasurement(const int& dictionary, const std::size_t dictionary_offset, const std::size_t number_x, const std::size_t number_y, const double& marker_side, const double& inter_marker_length, std::shared_ptr<Camera> camera) :
    ArucoMeasurement(dictionary, std::move(camera))
{
    /* Setup board. */
    cv::Ptr<cv::aruco::GridBoard> gridboard = GridBoard::create(number_x, number_y, marker_side, inter_marker_length, get_dictionary());
    board_ = gridboard.staticCast<Board>();

    /* Apply dictionary offset. */
    for (auto& id : board_->ids)
        id += dictionary_offset;

    /* Setup detector with corner refinement.
       Alternatives are:
       - CORNER_REFINE_NONE
       - CORNER_REFINE_SUBPIX
       - CORNER_REFINE_CONTOUR
    */
    detector_parameters_ = DetectorParameters::create();
    detector_parameters_->cornerRefinementMethod = CORNER_REFINE_CONTOUR;

    /* Setup axis length for reference frame visualization. */
    axis_length_ = 0.5 * ((double)std::min(number_x, number_y) * (marker_side + inter_marker_length) + inter_marker_length);
}


ArucoBoardMeasurement::~ArucoBoardMeasurement()
{}


bool ArucoBoardMeasurement::freeze(const Data& data)
{
    /* Freeze measurements. */
    if (!ArucoMeasurement::freeze(data))
        return false;

    /* Get image .*/
    cv::Mat image = get_frozen_rgb_image();

    /* Perform markers detection. */
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> inliers;
    std::vector<std::vector<cv::Point2f>> outliers;
    detectMarkers(image, get_dictionary(), inliers, ids, detector_parameters_, outliers);

    /* Perform markers refinement. */
    refineDetectedMarkers(image, board_, inliers, ids, outliers, get_camera_intrinsic(), get_camera_distortion());

    if (ids.size() > 0)
    {
        /* Perform board pose estimation. */
        cv::Mat position;
        cv::Mat orientation;
        int outcome = estimatePoseBoard(inliers, ids, board_, get_camera_intrinsic(), get_camera_distortion(), orientation, position);

        /* Generate image for probe if setup. */
        if(is_probe("image_output"))
        {
            probe_image_ = image.clone();
            drawDetectedMarkers(probe_image_, inliers, ids);
            if (outcome > 0)
                drawAxis(probe_image_, get_camera_intrinsic(), get_camera_distortion(), orientation, position, axis_length_);

            get_probe("image_output").set_data(probe_image_);
        }

        if (outcome > 0)
        {
            set_pose(position, orientation);
            return true;
        }

        return false;
    }

    return false;
}
