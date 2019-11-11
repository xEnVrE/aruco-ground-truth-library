/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoMeasurement.h>
#include <ArucoBoardMeasurement.h>
#include <ArucoMarkerMeasurement.h>
#include <ThreePointReverseLinkMeasurement.h>

#include <BayesFilters/FilteringAlgorithm.h>

#include <Eigen/Dense>

#include <iCubCamera.h>
#include <iCubCameraRelative.h>
#include <iCubCameraRelativeExternal.h>

#include <YarpImageOfProbe.hpp>
#include <YarpVectorOfProbe.hpp>

#include <opencv2/aruco.hpp>

#include <cstdlib>
#include <string>


class TestExtrinsicAruco : public bfl::FilteringAlgorithm
{
public:
    TestExtrinsicAruco(const std::string& data_path, const std::string& type) :
        data_path_(data_path),
        type_(type)
    {
        if (data_path_.back() != '/')
            data_path_ += "/";
    }


    bool skip(const std::string&, const bool) override
    {
        return false;
    }

protected:
    bool initialization() override
    {
        if ((type_ != "board_0") && (type_ != "board_1"))
            throw(std::runtime_error("ReverseArucoMeasurement::initialization. Error: unknown type " + type_ + "."));

        const std::string port_prefix = "test-extrinsic-aruco/" + type_;

        camera_l_ = std::shared_ptr<iCubCamera>
        (
            new iCubCamera(data_path_ + "left-camera", 640, 480, 468.672, 323.045, 467.73, 245.784, true)
        );

        camera_l2r_ = std::shared_ptr<iCubCameraRelative>
        (
            new iCubCameraRelative(std::string("right"), data_path_ + "left-camera", data_path_ + "right-camera", 640, 480,
                                   468.672, 323.045, 467.73, 245.784, 468.488, 301.274, 467.427, 245.503, true)
        );

        /* Probes .*/
        pose_l_ = std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>>
        (
            new YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>("/" + port_prefix + "/pose_l:o")
        );

        pose_l2r_ = std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>>
        (
            new YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>("/" + port_prefix + "/pose_l2r:o")
        );

        /* aruco measurement. */
        std::unique_ptr<ArucoMeasurement> aruco_l;
        std::unique_ptr<ArucoMeasurement> aruco_l2r;
        std::size_t n_x = 2;
        std::size_t n_y = 2;
        std::size_t dictionary_offset = 0;
        const double intra_marker = 0.005;
        const double marker_length = 0.02;

        if (type_ == "board_1")
        {
            n_x = 1;
            n_y = 2;
            dictionary_offset = 4;
        }

        aruco_l = std::unique_ptr<ArucoBoardMeasurement>
        (
            new ArucoBoardMeasurement(cv::aruco::DICT_4X4_50, dictionary_offset, n_x, n_y, marker_length, intra_marker, camera_l_)
        );
        aruco_l2r = std::unique_ptr<ArucoBoardMeasurement>
        (
            new ArucoBoardMeasurement(cv::aruco::DICT_4X4_50, dictionary_offset, n_x, n_y, marker_length, intra_marker, camera_l2r_)
        );

        /* Three point reverse link measurement. */
        Eigen::Vector3d point_0(-0.0303521, 0.0243051, 0.0389612);
        Eigen::Vector3d point_1(-0.0303521, -0.0256949, 0.0389612);
        Eigen::Vector3d point_2(0.0196458, 0.0243051, 0.0384987);
        Eigen::Vector3d corner_offset(0.0025, 0.0025, 0.0);
        if (type_ == "board_1")
        {
            point_0 = Eigen::Vector3d(-0.0437343, 0.000461959, -0.00474314);
            point_1 = Eigen::Vector3d(-0.0437343, 0.0254620, -0.00474314);
            point_2 = Eigen::Vector3d(0.00626574, 0.000461959, -0.00474314);
        }

        link_l_ = std::unique_ptr<ThreePointReverseLinkMeasurement>
        (
            new ThreePointReverseLinkMeasurement(point_0, point_1, point_2, corner_offset, std::move(aruco_l))
        );
        link_l2r_ = std::unique_ptr<ThreePointReverseLinkMeasurement>
        (
            new ThreePointReverseLinkMeasurement(point_0, point_1, point_2, corner_offset, std::move(aruco_l2r))
        );
        link_l_->set_probe("pose", std::move(pose_l_));
        link_l2r_->set_probe("pose_w_camera", std::move(pose_l2r_));

        return true;
    }


    void filteringStep() override
    {
        std::string key;
        getline(std::cin, key);
        bool execute = false;

        /* Get keyboard left/right arrows.*/
        if (key == "n")
            execute = true;
        else if (key == "b")
        {
            int frame_id = camera_l_->get_frame();
            camera_l_->set_frame(frame_id - 2);
            camera_l2r_->set_frame(frame_id - 2);

            execute = true;
        }
        else if (key == "q")
            teardown();

        /* Freeze aruco measurements. */
        if (execute)
        {
            link_l_->freeze();
            link_l2r_->freeze();
        }
    }


    bool runCondition() override
    {
        return true;
    }

private:
    const std::string type_;

    std::string data_path_;

    std::unique_ptr<ReverseLinkMeasurement> link_l_;

    std::unique_ptr<ReverseLinkMeasurement> link_l2r_;

    std::shared_ptr<iCubCamera> camera_l_;

    std::shared_ptr<iCubCamera> camera_l2r_;

    std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>> pose_l_;

    std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>> pose_l2r_;
};


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Synopsis: test-extrinsic-aruco <data_path> <type>"  << std::endl;
        std::cout << "          <type> can be 'board_0' or 'board_1'"  << std::endl;
        std::cout << "          <use_external_reference> is required if <use_relative_camera> = true"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string data_path = std::string(argv[1]);
    const std::string type = std::string(argv[2]);

    TestExtrinsicAruco test(data_path, type);
    test.boot();
    test.run();
    if (!test.wait())
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
