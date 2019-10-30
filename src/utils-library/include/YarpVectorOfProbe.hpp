/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef YARPVECTOROFPROBE_H
#define YARPVECTOROFPROBE_H

#include <Data.h>
#include <any.h>

#include <Eigen/Dense>

#include <Probe.h>
#include <YarpBufferedPort.hpp>

#include <string>

#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>



template <class T, class U = yarp::sig::VectorOf<T>>
class YarpVectorOfProbe : public YarpBufferedPort<yarp::sig::VectorOf<T>>,
                          public Probe
{
public:
    YarpVectorOfProbe(const std::string& port_name);

    virtual ~YarpVectorOfProbe();

protected:
    void on_new_data() override;

private:
    yarp::sig::VectorOf<T> convert_from(const U& data);

    yarp::sig::VectorOf<T> data_;

    const std::string log_name_ = "YarpVectorOfProbe";
};


template <class T, class U>
YarpVectorOfProbe<T, U>::YarpVectorOfProbe(const std::string& port_name) :
    YarpBufferedPort<yarp::sig::VectorOf<T>>(port_name)
{}


template <class T, class U>
YarpVectorOfProbe<T, U>::~YarpVectorOfProbe()
{}


template <class T, class U>
void YarpVectorOfProbe<T, U>::on_new_data()
{
    data_ = convert_from(bfl::any::any_cast<U>(get_data()));

    this->send_data(data_);
}


template <class T, class U>
yarp::sig::VectorOf<T> YarpVectorOfProbe<T, U>::convert_from(const U& data)
{
    return data;
}


template <>
yarp::sig::VectorOf<double> YarpVectorOfProbe<double, Eigen::VectorXd>::convert_from(const Eigen::VectorXd& data)
{
    yarp::sig::VectorOf<double> tmp(data.size());
    yarp::eigen::toEigen(tmp) = data;

    return tmp;
}


template <>
yarp::sig::VectorOf<double> YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>::convert_from(const Eigen::Transform<double, 3, Eigen::Affine>& data)
{
    /* Assume by default transformation to x-y-z-axis-angle. */
    yarp::sig::VectorOf<double> tmp(7);
    yarp::eigen::toEigen(tmp).head<3>() = data.translation();

    Eigen::AngleAxisd axis_angle(data.rotation());
    yarp::eigen::toEigen(tmp).segment<3>(3) = axis_angle.axis();
    yarp::eigen::toEigen(tmp)(6) = axis_angle.angle();

    return tmp;
}

#endif /* YARPVECTOROFPROBE_H */
