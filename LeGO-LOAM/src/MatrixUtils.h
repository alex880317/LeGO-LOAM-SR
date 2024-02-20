#ifndef MATRIXUTILS_H
#define MATRIXUTILS_H

#include "Eigen/Dense"
#include "Eigen/Geometry"

class MatrixUtils {
public:
    // 构造函数
    MatrixUtils() = default;

    // 静态成员函数
    static Eigen::Matrix3d vectorToCrossMatrix(const Eigen::Vector3d& vec);
    static Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond& quaternion);
};

#endif // MATRIXUTILS_H
