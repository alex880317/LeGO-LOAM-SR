#include "functions.h"
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

namespace gtsamexpressions
{

    gtsam::Point2 projectGroundPlane(
        const gtsam::Pose3 &pose,
        const gtsam::Vector3 &measuredNormal,
        double measuredDistance,
        gtsam::OptionalJacobian<2, 6> H)
    {
        // 取得旋轉矩陣和平移向量
        gtsam::Matrix3 R_k_W = pose.rotation().matrix();
        gtsam::Vector3 t_k_W = pose.translation();

        // 計算變換後的法向量
        // gtsam::Vector3 transformedNormal = R_k_W * measuredNormal;
        gtsam::Vector3 transformedNormal(0.0, 0.0, 1.0);

        // 參數化法向量與距離
        double theta = std::atan2(transformedNormal.y(), transformedNormal.x());
        double phi = std::acos(transformedNormal.z() / transformedNormal.norm());
        double d_k_prime = measuredDistance + transformedNormal.dot(t_k_W);


        // std::cout << "Transformed Normal: [" 
        //       << transformedNormal(0) << ", " 
        //       << transformedNormal(1) << ", " 
        //       << transformedNormal(2) << "]" << std::endl;
        // std::cout << "Translation vector t_k_W : " << t_k_W.transpose() << std::endl;
        // std::cout << "dot : " << transformedNormal.dot(t_k_W) << std::endl;
        // std::cout << "d_k_prime (距離): " << d_k_prime << std::endl;

        // // 計算雅可比矩陣（如果需要）
        // if (H) {
        //     // 這裡的H是3x6的雅可比矩陣，具體計算取決於轉換過程
        //     H->setZero(); // 填入具體的雅可比計算邏輯
        // }

        // std::cout << "tau = " << gtsam::Vector3(theta, phi, d_k_prime) << std::endl;

        // 返回參數化的地面法向量和距離
        return gtsam::Point2(phi, d_k_prime - 1.78);
    }

} // namespace gtsamexpressions
