/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoMeasurement.h>
#include <ArucoBoardMeasurement.h>
#include <ArucoMarkerMeasurement.h>
#include <ReverseLinkMeasurement.h>

#include <BayesFilters/FilteringAlgorithm.h>

#include <iCubCamera.h>

#include <YarpImageOfProbe.hpp>
#include <YarpVectorOfProbe.hpp>

#include <opencv2/aruco.hpp>

#include <cstdlib>
#include <stdexcept>
#include <string>
#include "ReverseLinkMeasurement.h"


class ReverseArucoMeasurement : public bfl::FilteringAlgorithm
{
public:
    ReverseArucoMeasurement(const std::string& type, const std::string& laterality) :
        type_(type),
        laterality_(laterality)
    {}


    bool skip(const std::string&, const bool) override
    {
        return false;
    }

protected:
    bool initialization() override
    {
        if ((type_ != "marker") && (type_ != "board"))
            throw(std::runtime_error("ReverseArucoMeasurement::initialization. Error: unknown type " + type_ + "."));

        /* Camera. */
        camera_ = std::unique_ptr<iCubCamera>
        (
            new iCubCamera(laterality_, "test-aruco-measurement/" + laterality_, "", "")
        );

        /* Probes .*/
        data_probe_ = std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>>
        (
            new YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>("/test-aruco-measurement/" + laterality_ + "/data:o")
        );

        image_probe_ = std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>>
        (
            new YarpImageOfProbe<yarp::sig::PixelRgb>("/test-aruco-measurement/" + laterality_ + "/image:o")
        );

        /* aruco measurement. */
        std::unique_ptr<ArucoMeasurement> aruco;
        if (type_ == "marker")
        {
            const double marker_length = 0.05;
            aruco = std::unique_ptr<ArucoMarkerMeasurement>
            (
                new ArucoMarkerMeasurement(cv::aruco::DICT_4X4_50, marker_length, std::move(camera_))
            );
        }
        else if (type_ == "board")
        {
            const std::size_t n_x = 2;
            const std::size_t n_y = 2;
            const double intra_marker = 0.005;
            const double marker_length = 0.02;
            aruco = std::unique_ptr<ArucoBoardMeasurement>
            (
                new ArucoBoardMeasurement(cv::aruco::DICT_4X4_50, n_x, n_y, marker_length, intra_marker, std::move(camera_))
            );
        }
        aruco->set_probe("image_output", std::move(image_probe_));

        /* Reverse link measurement. */
        link_measurement_ = std::unique_ptr<ReverseLinkMeasurement>
        (
            new ReverseLinkMeasurement(std::move(aruco))
        );
        link_measurement_->set_probe("data_output", std::move(data_probe_));

        return true;
    }


    void filteringStep() override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        link_measurement_->freeze();
    }


    bool runCondition() override
    {
        return true;
    }

private:
    const std::string type_;

    const std::string laterality_;

    std::unique_ptr<ReverseLinkMeasurement> link_measurement_;

    std::unique_ptr<iCubCamera> camera_;

    std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>> data_probe_;

    std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>> image_probe_;
};


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Synopsis: test-aruco-measurement <type> <laterality>"  << std::endl;
        std::cout << "          <type> can be 'marker' or 'board'"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string type = std::string(argv[1]);
    const std::string laterality = std::string(argv[2]);

    ReverseArucoMeasurement test(type, laterality);
    test.boot();
    test.run();
    if (!test.wait())
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
