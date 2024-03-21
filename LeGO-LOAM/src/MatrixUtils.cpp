#include "MatrixUtils.h"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <numeric>
#include <iostream>

Eigen::Matrix3d MatrixUtils::vectorToCrossMatrix(const Eigen::Vector3d& vec) {
    double a = vec(0);
    double b = vec(1);
    double c = vec(2);

    Eigen::Matrix3d cross_matrix;
    cross_matrix << 0, -c, b,
                    c, 0, -a,
                    -b, a, 0;

    return cross_matrix;
}

Eigen::Vector3d MatrixUtils::quaternionToEulerAngles(const Eigen::Quaterniond& quaternion) {
    // Eigen::Matrix3d R = quaternion.toRotationMatrix();
    double q_w = quaternion.w();
    Eigen::Vector3d q_v(quaternion.x(), quaternion.y(), quaternion.z());

    // 定义单位矩阵I
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // 计算旋转矩阵R
    Eigen::Matrix3d R = (q_w * q_w - q_v.dot(q_v)) * I
                        + 2 * q_v * q_v.transpose()
                        + 2 * q_w * MatrixUtils::vectorToCrossMatrix(q_v); // 注意：Eigen没有直接的vectorToCrossMatrix()成员函数，需要自定义

    double roll, pitch, yaw;
    pitch = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    yaw = std::atan2(R(1, 0) / std::cos(pitch), R(0, 0) / std::cos(pitch));
    roll = std::atan2(R(2, 1) / std::cos(pitch), R(2, 2) / std::cos(pitch));
    Eigen::Vector3d vec(roll, pitch, yaw);
    // std::cout << "yaw = " << yaw << std::endl;

    return vec;
}

double MatrixUtils::acot(double x) {
    return std::atan2(1.0, x);
}

double MatrixUtils::cot(double x) {
    return 1 / tan(x/1.0);
}

// template <typename T>
// int MatrixUtils::sign(T val) {
//     return (T(0) < val) - (val < T(0));
// }
int MatrixUtils::sign(double val) {
    return (0 < val) - (val < 0);
}

double MatrixUtils::calculateStd(const std::vector<double>& values) {
    double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    double sq_sum = std::inner_product(values.begin(), values.end(), values.begin(), 0.0,
                                       [](double const & x, double const & y) { return x + y; },
                                       [mean](double const & x, double const & y) { return (x - mean) * (y - mean); });
    return std::sqrt(sq_sum / values.size());
}

// 函式模板，用於將任意大小的Eigen矩陣轉換為向量
// template<typename T>
double* MatrixUtils::EigenMatrixToArray(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix) {
    // 将Eigen矩阵转换为std::vector
    std::vector<double> tempVector;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            tempVector.push_back(matrix(i, j));
        }
    }
    
    // 将std::vector转换为动态分配的数组
    double* array = new double[tempVector.size()];
    for (size_t i = 0; i < tempVector.size(); ++i) {
        array[i] = tempVector[i];
    }
    
    return array;
}

double MatrixUtils::calculateStandardDeviation(const std::vector<double>& vec) {
    double mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
    double sq_sum = std::accumulate(vec.begin(), vec.end(), 0.0,
                                    [mean](double acc, double val) {
                                        return acc + (val - mean) * (val - mean);
                                    });
    double variance = sq_sum / vec.size();
    return std::sqrt(variance);
}

// std::vector<double> EigenMatrixToVector(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix) {
//     std::vector<double> vector;
//     vector.reserve(matrix.rows() * matrix.cols()); // 预分配足够空间
//     for (int i = 0; i < matrix.rows(); ++i) {
//         for (int j = 0; j < matrix.cols(); ++j) {
//             vector.push_back(matrix(i, j));
//         }
//     }
//     return vector;
// }
