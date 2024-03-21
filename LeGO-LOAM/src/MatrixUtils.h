#ifndef MATRIXUTILS_H
#define MATRIXUTILS_H

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <vector>
#include <numeric>
#include <cmath>
// template<typename T>
class MatrixUtils {
public:
    // 构造函数
    MatrixUtils() = default;

    

    // 静态成员函数
    static Eigen::Matrix3d vectorToCrossMatrix(const Eigen::Vector3d& vec);
    static Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond& quaternion);
    static double acot(double x);
    static double cot(double x);
    // template <typename T>
    // static int sign(T val);
    static int sign(double val);
    static double calculateStd(const std::vector<double>& values);
    // template<typename T>
    // static T* EigenMatrixToArray(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& matrix);
    static double* EigenMatrixToArray(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix);
    static double calculateStandardDeviation(const std::vector<double>& vec);
};

#endif // MATRIXUTILS_H
