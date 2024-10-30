#ifndef EXPRESSIONS_H
#define EXPRESSIONS_H

#include "functions.h"
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>

namespace gtsamexpressions {

// 將 projectGroundPlane 函數包裝為 Expression
inline gtsam::Expression<gtsam::Vector3> projectGroundPlane_(
    const gtsam::Expression<gtsam::Pose3>& poseExpr,
    const gtsam::Expression<gtsam::Vector3>& measuredNormalExpr,
    const gtsam::Expression<double>& measuredDistanceExpr)
{
    return gtsam::Expression<gtsam::Vector3>(
        [](const gtsam::Pose3& pose, const gtsam::Vector3& measuredNormal, double measuredDistance,
           gtsam::OptionalJacobian<3, 6> H_pose = boost::none,
           gtsam::OptionalJacobian<3, 3> H_normal = boost::none,
           gtsam::OptionalJacobian<3, 1> H_distance = boost::none) {
            return projectGroundPlane(pose, measuredNormal, measuredDistance, H_pose);
        }, poseExpr, measuredNormalExpr, measuredDistanceExpr);
}

} // namespace gtsamexpressions

#endif // EXPRESSIONS_H
