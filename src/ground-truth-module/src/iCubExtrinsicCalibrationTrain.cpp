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
#include <vector>

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


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Synopsis: icub-extr-calib-train <data_path> <output>"  << std::endl;

        return EXIT_FAILURE;
    }
    const std::string data_path = std::string(argv[1]);
    const std::string output_path = std::string(argv[2]);

    /* Load data from file. */
    bool valid_data = false;
    Eigen::MatrixXd data;
    std::tie(valid_data, data) = read_data_from_file(data_path, 24);
    if (!valid_data)
    {
        std::cout << "Cannot load data from file "  << data_path << "." << std::endl;

        return EXIT_FAILURE;
    }

    /* Get input and output pairs. */
    Eigen::MatrixXd input = data.topRows(3);
    Eigen::MatrixXd output = data.bottomRows(7);

    /* Initialize SVM machine. */
    // double gamma = 1e-3;
    double gamma = 1e-1;
    double regularization = 1 / gamma;
    iCub::learningmachine::LSSVMLearner svm(3, 6, regularization);
    // svm.getKernel()->setGamma(500);

    /* Add samples. */
    for (std::size_t i = 0; i < input.cols(); i++)
    {
        /* Move output to se(3). */
        Eigen::AngleAxisd angle_axis(output.col(i)(6), output.col(i).segment<3>(3));
        Eigen::Matrix3d rotation = angle_axis.toRotationMatrix();
        double theta = std::acos((rotation.trace() - 1) / 2.0) + std::numeric_limits<double>::epsilon();
        Eigen::Matrix3d log_R = theta / (2 * std::sin(theta)) * (rotation - rotation.transpose());
        Eigen::Vector3d omega;
        omega(0) = log_R(2, 1);
        omega(1) = log_R(0, 2);
        omega(2) = log_R(1, 0);
        Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / (theta * theta) * log_R + (theta - std::sin(theta)) / (std::pow(theta, 3)) * log_R * log_R;
        Eigen::Vector3d t = V.inverse() * output.col(i).head<3>();

        Eigen::VectorXd se3(6);
        se3.head<3>() = t;
        se3.tail<3>() = omega;

        /* Convert to yarp vectors. */
        yarp::sig::Vector in(3);
        yarp::eigen::toEigen(in) = input.col(i);

        yarp::sig::Vector out(6);
        yarp::eigen::toEigen(out) = se3;

        /* Provide a sample. */
        svm.feedSample(in, out);
    }

    /* Train. */
    std::cout << "Training..." << std::flush;
    svm.train();
    std::cout << "done." << std::endl;

    /* Save on file. */
    std::ofstream out;
    out.open(output_path);
    if (!out.is_open())
    {
        std::cout << "Cannot save trained model on file." << std::endl;

        return EXIT_FAILURE;
    }
    yarp::os::Bottle model;
    svm.writeBottle(model);
    out << model.toString();
    out.close();

    return EXIT_SUCCESS;
}
