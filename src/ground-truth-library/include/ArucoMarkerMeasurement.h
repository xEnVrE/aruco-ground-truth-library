/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ARUCOMARKERMEASUREMENT_H
#define ARUCOMARKERMEASUREMENT_H

#include <ArucoMeasurement.h>
#include <Camera.h>

#include <memory>

class ArucoMarkerMeasurement : public ArucoMeasurement
{
public:
    ArucoMarkerMeasurement(const int& dictionary, const double& marker_length, std::shared_ptr<Camera> camera);

    virtual ~ArucoMarkerMeasurement();

    bool freeze(const bfl::Data& data = bfl::Data()) override;

private:
    double marker_length_;

    /* For visualization purposes. */
    cv::Mat probe_image_;
};

#endif /* ARUCOMARKERMEASUREMENT_H */
