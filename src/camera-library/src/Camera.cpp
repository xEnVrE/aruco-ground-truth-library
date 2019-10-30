/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Camera.h>

#include <iostream>

using namespace Eigen;


Camera::~Camera()
{ }


bool Camera::initialize()
{
    bool ok = true;

    /* Cache the deprojection matrix once for all. */
    ok &= evaluate_deprojection_matrix();

    return ok;
}


bool Camera::step_frame()
{
    /* Not implemented by default. */
    throw(std::runtime_error(log_name_ + "::step_frame. Not implemented in base class."));
}


std::size_t Camera::get_frame() const
{
    /* Return 0 by default. */
    return 0;
}


bool Camera::set_frame(const std::size_t& index)
{
    /* Not implemented by default. */
    throw(std::runtime_error(log_name_ + "::reset. Not implemented in base class."));
}


bool Camera::reset()
{
    /* Not implemented by default. */
    throw(std::runtime_error(log_name_ + "::reset. Not implemented in base class."));
}


std::pair<bool, MatrixXd> Camera::get_deprojection_matrix() const
{
    if (!deprojection_matrix_initialized_)
        return std::make_pair(false, MatrixXd());

    return std::make_pair(true, deprojection_matrix_);
}


bool Camera::evaluate_deprojection_matrix()
{
    if (!parameters_.is_initialized())
        throw(std::runtime_error(log_name_ + "::reset. Camera parameters not initialized. Did you initialize the class member 'parameters_' in the derived class?."));

    /* Allocate storage. */
    deprojection_matrix_.resize(3, parameters_.width * parameters_.height);

    // Evaluate deprojection matrix
    int i = 0;
    for (std::size_t u = 0; u < parameters_.width; u++)
    {
        for (std::size_t v = 0; v < parameters_.height; v++)
        {
            deprojection_matrix_(0, i) = (u - parameters_.cx) / parameters_.fx;
            deprojection_matrix_(1, i) = (v - parameters_.cy) / parameters_.fy;
            deprojection_matrix_(2, i) = 1.0;

            i++;
        }
    }

    deprojection_matrix_initialized_ = true;

    return true;
}


std::pair<bool, CameraParameters> Camera::get_parameters() const
{
    if (!parameters_.is_initialized())
        return std::make_pair(false, CameraParameters());

    return std::make_pair(true, parameters_);
}
