/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ARUCOMEASUREMENT_H
#define ARUCOMEASUREMENT_H

#include <BayesFilters/LinearMeasurementModel.h>

#include <Camera.h>
#include <ProbeContainer.h>

#include <Eigen/Dense>

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class ArucoMeasurement : public bfl::MeasurementModel,
                         public ProbeContainer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ArucoMeasurement(const int& dictionary, std::unique_ptr<Camera> camera);

    virtual ~ArucoMeasurement();

    virtual bool freeze(const bfl::Data& data = bfl::Data()) override;

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const override;

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

protected:
    cv::Mat get_camera_intrinsic() const;

    cv::Mat get_camera_distortion() const;

    Eigen::Transform<double, 3, Eigen::Affine> get_frozen_camera_pose() const;

    cv::Mat get_frozen_rgb_image() const;

    const std::size_t& get_width() const;

    const std::size_t& get_height() const;

    cv::Ptr<cv::aruco::Dictionary> get_dictionary();

    void set_pose(cv::Mat position, cv::Mat orientation);

    void set_pose(cv::Vec3d position, cv::Vec3d orientation);

private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    /* Camera. */

    std::unique_ptr<Camera> camera_;

    cv::Mat cam_intrinsic_;

    cv::Mat cam_distortion_;

    std::size_t width_;

    std::size_t height_;

    /* Storage for pose. */

    Eigen::Transform<double, 3, Eigen::Affine> pose_;

    bool valid_pose_ = false;

    /* Frozen quantities. */

    cv::Mat camera_rgb_image_;

    Eigen::Transform<double, 3, Eigen::Affine> camera_pose_;

    std::vector<int> ids_;

    std::vector<std::vector<cv::Point2f>> inliers_;

    std::vector<std::vector<cv::Point2f>> outliers_;

    const std::string log_name_ = "ArucoMeasurement";
};

#endif /* ARUCOMEASUREMENT_H */
