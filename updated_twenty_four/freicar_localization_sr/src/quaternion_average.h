//
// Created by freicar on 3/8/21.
//

#ifndef FREICAR_LOCALIZATION_QUATERNION_AVERAGE_H
#define FREICAR_LOCALIZATION_QUATERNION_AVERAGE_H


#include <Eigen/SVD>
#include <vector>
#include <iostream>


/// Method to find the average of a set of rotation quaternions using Singular Value Decomposition


Eigen::Transform<float, 3, 2, 0>::LinearMatrixType rotationAverage(std::vector<Eigen::Transform<float, 3, 2, 0>::LinearMatrixType> quaternions)
{

    if (quaternions.size() == 0)
    {
        std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
        return Eigen::Transform<float, 3, 2, 0>::LinearMatrixType::Zero();
    }

    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
//    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
    Eigen::Transform<float, 3, 2, 0>::LinearMatrixType A;
//
    for (int q=0; q<quaternions.size(); ++q)
        A += quaternions[q] * quaternions[q].transpose();

    std::cout << "size of A" <<A.size()<<std::endl;

    // normalise with the number of quaternions
    A /= quaternions.size();
//
//    // Compute the SVD of this 4x4 matrix
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
//
    Eigen::VectorXf singularValues = svd.singularValues();
    Eigen::MatrixXf U = svd.matrixU();

    // find the eigen vector corresponding to the largest eigen value
    int largestEigenValueIndex;
    float largestEigenValue;
    bool first = true;

    for (int i=0; i<singularValues.rows(); ++i)
    {
        if (first)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
            first = false;
        }
        else if (singularValues(i) > largestEigenValue)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
        }
    }
//
    Eigen::Transform<float, 3, 2, 0>::LinearMatrixType average;
//    Eigen::Vector4f average;
    average(0) = U(0, largestEigenValueIndex);
    average(1) = U(1, largestEigenValueIndex);
    average(2) = U(2, largestEigenValueIndex);
    average(3) = U(3, largestEigenValueIndex);
//
    return average;
}

//
//Eigen::Vector4f quaternionAverage(std::vector<Eigen::Vector4f> quaternions)
//{
//
//    if (quaternions.size() == 0)
//    {
//        std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
//        return Eigen::Vector4f::Zero();
//    }
//
//    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
//    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
//
//    for (int q=0; q<quaternions.size(); ++q)
//        A += quaternions[q] * quaternions[q].transpose();
//
//    // normalise with the number of quaternions
//    A /= quaternions.size();
//
//    // Compute the SVD of this 4x4 matrix
//    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
//
//    Eigen::VectorXf singularValues = svd.singularValues();
//    Eigen::MatrixXf U = svd.matrixU();
//
//    // find the eigen vector corresponding to the largest eigen value
//    int largestEigenValueIndex;
//    float largestEigenValue;
//    bool first = true;
//
//    for (int i=0; i<singularValues.rows(); ++i)
//    {
//        if (first)
//        {
//            largestEigenValue = singularValues(i);
//            largestEigenValueIndex = i;
//            first = false;
//        }
//        else if (singularValues(i) > largestEigenValue)
//        {
//            largestEigenValue = singularValues(i);
//            largestEigenValueIndex = i;
//        }
//    }
//
//    Eigen::Vector4f average;
//    average(0) = U(0, largestEigenValueIndex);
//    average(1) = U(1, largestEigenValueIndex);
//    average(2) = U(2, largestEigenValueIndex);
//    average(3) = U(3, largestEigenValueIndex);
//
//    return average;
//}

#endif //FREICAR_LOCALIZATION_QUATERNION_AVERAGE_H