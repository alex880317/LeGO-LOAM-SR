#include "MatrixUtils.h"

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
    Eigen::Matrix3d R = quaternion.toRotationMatrix();

    double roll, pitch, yaw;
    pitch = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    yaw = std::atan2(R(1, 0) / std::cos(pitch), R(0, 0) / std::cos(pitch));
    roll = std::atan2(R(2, 1) / std::cos(pitch), R(2, 2) / std::cos(pitch));

    return Eigen::Vector3d(roll, pitch, yaw);
}
