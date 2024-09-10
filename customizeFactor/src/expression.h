#pragma once

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>



inline gtsam::Vector3_ expressGroundPlane_(const gtsam::Pose3_& pose, const gtsam::Point3_& measuredNormal, double measuredDistance) {
    // gtsam::Matrix3 R_k_W = pose.rotation().matrix();
    // gtsam::Vector3 measuredNormal_W = R_k_W * measuredNormal;

    // // 計算誤差 (用來代替之前的手動計算)
    // double theta = std::atan2(measuredNormal_W.y(), measuredNormal_W.x());  // 方位角
    // double phi = std::atan2(measuredNormal_W.z(), measuredNormal_W.head<2>().norm());  // 俯仰角
    // double d_k_prime = measuredDistance - (pose.translation().transpose() * measuredNormal_W);

    return gtsam::Vector3_(&expressGroundPlane, );
}
