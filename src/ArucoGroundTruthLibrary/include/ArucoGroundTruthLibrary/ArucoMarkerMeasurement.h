/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ARUCOGROUNDTRUTHLIBRARY_ARUCOMARKERMEASUREMENT_H
#define ARUCOGROUNDTRUTHLIBRARY_ARUCOMARKERMEASUREMENT_H

#include <ArucoGroundTruthLibrary/ArucoMeasurement.h>

#include <BayesFilters/Data.h>

#include <RobotsIO/Camera/Camera.h>

#include <memory>

namespace ArucoGroundTruthLibrary {
    class ArucoMarkerMeasurement;
}


class ArucoGroundTruthLibrary::ArucoMarkerMeasurement : public ArucoGroundTruthLibrary::ArucoMeasurement
{
public:
    ArucoMarkerMeasurement(const int& dictionary, const double& marker_length, std::shared_ptr<RobotsIO::Camera::Camera> camera);

    virtual ~ArucoMarkerMeasurement();

    bool freeze(const bfl::Data& data = bfl::Data()) override;

private:
    double marker_length_;

    /* For visualization purposes. */
    cv::Mat probe_image_;
};

#endif /* ARUCOGROUNDTRUTHLIBRARY_ARUCOMARKERMEASUREMENT_H */
