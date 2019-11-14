/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ARUCOBOARDMEASUREMENT_H
#define ARUCOBOARDMEASUREMENT_H

#include <ArucoMeasurement.h>

#include <BayesFilters/Data.h>

#include <RobotsIO/Camera/Camera.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <memory>
#include <string>


class ArucoBoardMeasurement : public ArucoMeasurement
{
public:
    ArucoBoardMeasurement(const int& dictionary, const std::size_t dictionary_offset, const std::size_t number_x, const std::size_t number_y, const double& marker_side, const double& inter_marker_length, std::shared_ptr<RobotsIO::Camera::Camera> camera);

    virtual ~ArucoBoardMeasurement();

    bool freeze(const bfl::Data& data = bfl::Data()) override;

private:
    cv::Ptr<cv::aruco::Board> board_;

    cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;

    /* For visualization purposes. */
    double axis_length_;

    cv::Mat probe_image_;

    const std::string log_name_ = "ArucoBoardMeasurement";
};

#endif /* ARUCOBOARDMEASUREMENT_H */
