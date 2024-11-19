#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>


namespace gtsamexpressions {

// 將 Pose3 投影為地面參數化法向量和距離的函數
gtsam::Point2 projectGroundPlane(const gtsam::Pose3& pose, const gtsam::Vector3& measuredNormal, double measuredDistance, gtsam::OptionalJacobian<2, 6> H = boost::none);
    
} // namespace gtsamexpressions


#endif // FUNCTIONS_H
