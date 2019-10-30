/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef YARPIMAGEOFPROBE_H
#define YARPIMAGEOFPROBE_H

#include <Data.h>
#include <any.h>

#include <Eigen/Dense>

#include <Probe.h>
#include <YarpBufferedPort.hpp>

#include <string>

#include <yarp/cv/Cv.h>
#include <yarp/sig/Image.h>


template <class T>
class YarpImageOfProbe : public YarpBufferedPort<yarp::sig::ImageOf<T>>,
                         public Probe
{
public:
    YarpImageOfProbe(const std::string& port_name);

    virtual ~YarpImageOfProbe();

protected:
    void on_new_data() override;

private:
    cv::Mat data_cv_;

    yarp::sig::ImageOf<T> data_;

    const std::string log_name_ = "YarpImageOfProbe";
};


template <class T>
YarpImageOfProbe<T>::YarpImageOfProbe(const std::string& port_name) :
    YarpBufferedPort<yarp::sig::ImageOf<T>>(port_name)
{}


template <class T>
YarpImageOfProbe<T>::~YarpImageOfProbe()
{}


template <class T>
void YarpImageOfProbe<T>::on_new_data()
{
    data_cv_ = bfl::any::any_cast<cv::Mat>(get_data());

    data_ = yarp::cv::fromCvMat<T>(data_cv_);

    this->send_data(data_);
}


#endif /* YARPIMAGEOFPROBE_H */
