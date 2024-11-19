#ifndef EXPRESSIONS_H
#define EXPRESSIONS_H

#include "functions.h"
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>

#include <iostream>

namespace gtsamexpressions {

// 將 projectGroundPlane 函數包裝為 Expression
inline gtsam::Point2_ projectGroundPlane_(
    const gtsam::Pose3_& poseExpr,
    const gtsam::Vector3& measuredNormal,  // 直接傳入固定的觀測量
    double measuredDistance)               // 直接傳入固定的觀測量
{

    // 使用 lambda 表達式封裝 projectGroundPlane 函數
    return gtsam::Point2_(
        [measuredNormal, measuredDistance](const gtsam::Pose3& pose,
                                           gtsam::OptionalJacobian<2, 6> H_pose = boost::none) {
            // 使用固定觀測量計算
            return projectGroundPlane(pose, measuredNormal, measuredDistance);
        },
        poseExpr);

    // // 創建一個函數對象，將measuredNormal和measuredDistance綁定
    // struct ProjectGroundPlaneFunctor {
    //     const gtsam::Vector3& n_;
    //     double d_;
        
    //     ProjectGroundPlaneFunctor(const gtsam::Vector3& n, double d) 
    //         : n_(n), d_(d) {}
            
    //     gtsam::Point2 operator()(const gtsam::Pose3& pose, 
    //         gtsam::OptionalJacobian<2,6> H = boost::none) const {
    //         return projectGroundPlane(pose, n_, d_, H);
    //     }
    // };
    
    // // 使用函數對象而不是lambda
    // return gtsam::Point2_(
    //     ProjectGroundPlaneFunctor(measuredNormal, measuredDistance),
    //     poseExpr);
}

// 測試函數，用於顯示 "expressions.h" 被包含的訊息
inline void displayExpressionsHeader() {
    std::cout << "expressions.h is included successfully." << std::endl;
}

} // namespace gtsamexpressions


#endif // EXPRESSIONS_H
