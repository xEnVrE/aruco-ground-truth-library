/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Eigen/Dense>

#include <iCub/learningMachine/LSSVMLearner.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

#include <unsupported/Eigen/MatrixFunctions>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>


std::pair<bool, Eigen::MatrixXd> read_data_from_file(const std::string& file_name, const std::size_t& number_of_fields)
{
    Eigen::MatrixXd data;

    std::ifstream istrm(file_name);
    if (!istrm.is_open())
        return std::make_pair(false, Eigen::MatrixXd(0,0));
    else
    {
        std::vector<std::string> istrm_strings;
        std::string line;
        while (std::getline(istrm, line))
        {
            istrm_strings.push_back(line);
        }

        data.resize(number_of_fields, istrm_strings.size());
        std::size_t found_lines = 0;
        for (auto line : istrm_strings)
        {
            std::size_t found_fields = 0;
            std::string number_str;
            std::istringstream iss(line);

            while (iss >> number_str)
            {
                std::size_t index = (number_of_fields * found_lines) + found_fields;
                *(data.data() + index) = std::stod(number_str);
                found_fields++;
            }
            if (number_of_fields != found_fields)
                return std::make_pair(false, Eigen::MatrixXd(0,0));

            found_lines++;
        }

        istrm.close();

        return std::make_pair(true, data);
    }
}


Eigen::Transform<double, 3, Eigen::Affine> exp_map(const Eigen::VectorXd& se3)
{
    Eigen::Transform<double, 3, Eigen::Affine> SE3;

    Eigen::Matrix3d log_R = Eigen::Matrix3d::Zero();
    log_R(0, 1) = -1.0 * se3(5);
    log_R(0, 2) = se3(4);
    log_R(1, 0) = se3(5);
    log_R(1, 2) = -1.0 * se3(3);
    log_R(2, 0) = -1.0 * se3(4);
    log_R(2, 1) = se3(3);

    double theta = se3.tail<3>().norm() + std::numeric_limits<double>::epsilon();
    Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / (theta * theta) * log_R + (theta - std::sin(theta)) / (std::pow(theta, 3)) * log_R * log_R;

    SE3 = Eigen::Translation<double, 3>(V * se3.head<3>());
    SE3.rotate(log_R.exp());

    return SE3;
}


int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cout << "Synopsis: test-trained-model <train_data> <model> <predictions>"  << std::endl;

        return EXIT_FAILURE;
    }
    const std::string training_path = std::string(argv[1]);
    const std::string model_path = std::string(argv[2]);
    const std::string predictions_path = std::string(argv[3]);

    /* Get inputs from training data. */
    bool valid_data = false;
    Eigen::MatrixXd data;
    std::tie(valid_data, data) = read_data_from_file(training_path, 24);
    if (!valid_data)
    {
        std::cout << "Cannot load training data from file "  << training_path << "." << std::endl;

        return EXIT_FAILURE;
    }
    Eigen::MatrixXd input = data.topRows(3);

    /* Load model. */
    std::ifstream in;
    in.open(model_path);
    if (!in.is_open())
    {
        std::cout << "Cannot load model " << model_path << std::endl;

        return EXIT_FAILURE;
    }
    yarp::os::Bottle model;
    std::stringstream ss;
    ss << in.rdbuf();
    model.fromString(ss.str());
    in.close();

    /* Open output. */
    std::ofstream out;
    out.open(predictions_path);
    if (!out.is_open())
    {
        std::cout << "Cannot open output file " << predictions_path << std::endl;

        return EXIT_FAILURE;
    }

    /* Initialize SVM machine. */
    iCub::learningmachine::LSSVMLearner svm;
    svm.readBottle(model);

    /* Eigen precision format .*/
    Eigen::IOFormat full_precision(Eigen::FullPrecision);

    /* Make predictions. */
    Eigen::VectorXd place_holder = Eigen::VectorXd::Zero(14);
    for (std::size_t i = 0; i < input.cols(); i++)
    {
        /* Set input. */
        yarp::sig::Vector in(3);
        yarp::eigen::toEigen(in) = input.col(i);

        /* Get prediction. */
        yarp::sig::Vector prediction = svm.predict(in).getPrediction();

        /* Convert to SE3. */
        Eigen::Transform<double, 3, Eigen::Affine> output = exp_map(yarp::eigen::toEigen(prediction));

        /* Get angle axis. */
        Eigen::AngleAxisd angle_axis(output.rotation());
        Eigen::VectorXd angle(1);
        angle(0) = angle_axis.angle();

        /* Write output. */
        out << input.col(i).transpose().format(full_precision) << " "
            << place_holder.transpose() << " "
            << output.translation().transpose().format(full_precision) << " "
            << angle_axis.axis().transpose().format(full_precision) << " "
            << angle.format(full_precision) << std::endl;
    }

    out.close();

    return EXIT_SUCCESS;
}
