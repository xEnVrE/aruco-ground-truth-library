/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoMeasurement.h>
#include <ArucoBoardMeasurement.h>
#include <ArucoMarkerMeasurement.h>

#include <BayesFilters/FilteringAlgorithm.h>

#include <RobotsIO/Camera/iCubCamera.h>
#include <RobotsIO/Utils/YarpImageOfProbe.hpp>
#include <RobotsIO/Utils/YarpVectorOfProbe.hpp>

#include <opencv2/aruco.hpp>

#include <cstdlib>
#include <stdexcept>
#include <string>

using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;


class TestArucoMeasurement : public bfl::FilteringAlgorithm
{
public:
    TestArucoMeasurement(const std::string& type) :
        type_(type)
    {}


    bool skip(const std::string&, const bool) override
    {
        return false;
    }

protected:
    bool initialization() override
    {
        if ((type_ != "marker") && (type_ != "board"))
            throw(std::runtime_error("TestArucoMeasurement::initialization. Error: unknown type " + type_ + "."));

        /* Camera. */
        camera_ = std::unique_ptr<iCubCamera>
        (
            new iCubCamera("icub", "right", "test-aruco-measurement", "", "")
        );

        /* Probes .*/
        pose_probe_ = std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>>
        (
            new YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>("/test-aruco-measurement/data:o")
        );

        image_probe_ = std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>>
        (
            new YarpImageOfProbe<yarp::sig::PixelRgb>("/test-aruco-measurement/image:o")
        );

        /* aruco measurement. */
        if (type_ == "marker")
        {
            const double marker_length = 0.05;
            aruco_ = std::unique_ptr<ArucoMarkerMeasurement>
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
            aruco_ = std::unique_ptr<ArucoBoardMeasurement>
            (
                new ArucoBoardMeasurement(cv::aruco::DICT_4X4_50, 0, n_x, n_y, marker_length, intra_marker, std::move(camera_))
            );
        }

        aruco_->set_probe("pose_w_camera", std::move(pose_probe_));
        aruco_->set_probe("image_output", std::move(image_probe_));

        return true;
    }


    void filteringStep() override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        aruco_->freeze();
    }


    bool runCondition() override
    {
        return true;
    }

private:
    const std::string type_;

    std::unique_ptr<ArucoMeasurement> aruco_;

    std::unique_ptr<iCubCamera> camera_;

    std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>> pose_probe_;

    std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>> image_probe_;
};


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Synopsis: test-aruco-measurement <type>"  << std::endl;
        std::cout << "          <type> can be 'marker' or 'board'"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string type = std::string(argv[1]);

    TestArucoMeasurement test(type);
    test.boot();
    test.run();
    if (!test.wait())
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
