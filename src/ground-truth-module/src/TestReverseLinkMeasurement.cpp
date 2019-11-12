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


class ReverseArucoMeasurement : public bfl::FilteringAlgorithm
{
public:
    ReverseArucoMeasurement(const std::string& type, const std::string& laterality, const bool& use_relative_camera, const bool& use_extrinsic_compensation, const std::string& model_path) :
        type_(type),
        laterality_(laterality),
        use_relative_camera_(use_relative_camera),
        use_extrinsic_compensation_(use_extrinsic_compensation),
        model_path_(model_path)
    {}


    bool skip(const std::string&, const bool) override
    {
        return false;
    }

protected:
    bool initialization() override
    {
        if ((type_ != "marker") && (type_ != "board_0") && (type_ != "board_1"))
            throw(std::runtime_error("ReverseArucoMeasurement::initialization. Error: unknown type " + type_ + "."));

        const std::string relative_postfix = use_relative_camera_ ? "_relative" : "";
        const std::string compensated_postfix = use_extrinsic_compensation_ ? "_compensated" : "";
        const std::string port_prefix = "test-aruco-measurement/" + type_ + "/" + laterality_ + relative_postfix + compensated_postfix;

        /* Camera. */
        if (use_relative_camera_)
        {
            camera_ = std::unique_ptr<iCubCameraRelative>
            (
                new iCubCameraRelative("icub", laterality_, port_prefix, "", "", use_extrinsic_compensation_, model_path_)
            );
        }
        else
            camera_ = std::unique_ptr<iCubCamera>
            (
                new iCubCamera("icub", laterality_, port_prefix, "", "", use_extrinsic_compensation_, model_path_)
            );

        /* Probes .*/
        pose_probe_ = std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>>
        (
            new YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>("/" + port_prefix + "/pose:o")
        );

        pose_w_camera_probe_ = std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>>
        (
            new YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>("/" + port_prefix + "/pose_w_camera:o")
        );

        image_probe_ = std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>>
        (
            new YarpImageOfProbe<yarp::sig::PixelRgb>("/" + port_prefix + "/image:o")
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
        else if ((type_ == "board_0") || (type_ == "board_1"))
        {
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

            aruco = std::unique_ptr<ArucoBoardMeasurement>
            (
                new ArucoBoardMeasurement(cv::aruco::DICT_4X4_50, dictionary_offset, n_x, n_y, marker_length, intra_marker, std::move(camera_))
            );
        }
        aruco->set_probe("image_output", std::move(image_probe_));

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

        link_measurement_ = std::unique_ptr<ThreePointReverseLinkMeasurement>
        (
            new ThreePointReverseLinkMeasurement(point_0, point_1, point_2, corner_offset, std::move(aruco))
        );
        link_measurement_->set_probe("pose", std::move(pose_probe_));
        link_measurement_->set_probe("pose_w_camera", std::move(pose_w_camera_probe_));

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

    const bool& use_relative_camera_ = false;

    const bool& use_extrinsic_compensation_ = false;

    const std::string& model_path_;

    std::unique_ptr<ReverseLinkMeasurement> link_measurement_;

    std::unique_ptr<iCubCamera> camera_;

    std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>> pose_probe_;

    std::unique_ptr<YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>> pose_w_camera_probe_;

    std::unique_ptr<YarpImageOfProbe<yarp::sig::PixelRgb>> image_probe_;
};


int main(int argc, char** argv)
{
    if ((argc < 5) || ((std::string(argv[4]) == "true") && argc != 6))
    {
        std::cout << "Synopsis: test-aruco-measurement <type> <laterality> <use_relative_camera> <use_extrinsic_compensation> [<model_path>]"  << std::endl;
        std::cout << "          <type> can be 'marker', 'board_0' or 'board_1'"  << std::endl;
        std::cout << "          <model_path> is required if <use_extrinsic_compensation> = true"  << std::endl;

        return EXIT_FAILURE;
    }

    const std::string type = std::string(argv[1]);
    const std::string laterality = std::string(argv[2]);
    const bool use_relative_camera = (std::string(argv[3]) == "true");
    bool use_extrinsic_compensation = (std::string(argv[4]) == "true");
    std::string model_path = "";
    if (use_extrinsic_compensation)
       model_path = std::string(argv[5]);

    ReverseArucoMeasurement test(type, laterality, use_relative_camera, use_extrinsic_compensation, model_path);
    test.boot();
    test.run();
    if (!test.wait())
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
