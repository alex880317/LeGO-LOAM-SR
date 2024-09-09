#ifndef GROUNDPLANEFACTOR_H
#define GROUNDPLANEFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>  // 引入GTSAM非線性因子基類
#include <gtsam/geometry/Pose3.h>             // 引入Pose3幾何類
#include <gtsam/geometry/Point3.h>            // 引入Point3幾何類
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>

class GroundPlaneFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
    gtsam::Vector3 measuredNormal_;    // 地面法向量測量
    double measuredDistance_;  // 地面距離測量
    

    // 合併的噪聲模型
    gtsam::SharedNoiseModel noiseModel_;

    GroundPlaneFactor(gtsam::Key key, const gtsam::Point3& normal, const double& distance,
                      const gtsam::SharedNoiseModel& noiseModel)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noiseModel, key),
          measuredNormal_(normal), measuredDistance_(distance), 
          noiseModel_(noiseModel) {}

    gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                                boost::optional<gtsam::Matrix&> H = boost::none) const override {
        // 計算距離誤差
        double initialDistance = 0;  

        // 計算法向量誤差
        gtsam::Vector3 initialNormal(0.0, 0.0, 1.0);  

        gtsam::Matrix3 R_k_W = pose.rotation().matrix();
        gtsam::Vector3 measuredNormal_W = R_k_W * measuredNormal_;

        // 計算法向量參數化 \(\tau(G^W_k)\) 
        double theta = std::atan2(measuredNormal_W.y(), measuredNormal_W.x());  // 方位角
        double phi = std::atan2(measuredNormal_W.z(), measuredNormal_W.head<3>().norm());  // 俯仰角
        double d_k_prime = measuredDistance_ - (pose.translation().transpose() * measuredNormal_W);  // 直接使用測量的距離值

        gtsam::Vector3 tau_measured(theta, phi, d_k_prime);  // 參數化後的測量值

        // 將預測的法向量進行參數化 
        double initial_theta = std::atan2(initialNormal.y(), initialNormal.x());
        double initial_phi = std::atan2(initialNormal.z(), initialNormal.head<3>().norm());
        double initial_d_k_prime = initialDistance;  

        gtsam::Vector3 tau_initial(initial_theta, initial_phi, initial_d_k_prime);  // 參數化後的預測值

        gtsam::Vector3 error = tau_measured - tau_initial;

        // 如果需要雅可比矩陣 H，則計算
        if (H) {
            H->setZero(3, 6);  // Jacobian 大小是 3x6

            gtsam::Vector3 G_k = measuredNormal_;
            gtsam::Vector3 G_k_W = measuredNormal_W;  // 假設地面法向量是 G_k_W
            
            // 構建雅可比矩陣
            gtsam::Matrix H_left(3, 4);
            H_left << -G_k_W(1) / (G_k_W(0) * G_k_W(0) + G_k_W(1) * G_k_W(1)), G_k_W(0) / (G_k_W(0) * G_k_W(0) + G_k_W(1) * G_k_W(1)), 0, 0,
                      -G_k_W(0) * G_k_W(2) / G_k_W.norm(), -G_k_W(1) * G_k_W(2) / G_k_W.norm(), (G_k_W(0) * G_k_W(0) + G_k_W(1) * G_k_W(1)) / G_k_W.norm(), 0,
                       0, 0, 0, 1;

            // 使用反對稱矩陣構建旋轉的雅可比
            gtsam::Matrix H_right(4, 6);
            gtsam::Matrix3 skew_RWGk = gtsam::skewSymmetric(R_k_W * G_k);

            gtsam::Rot3 SO3 = gtsam::Rot3(R_k_W);
            // 將李群轉換為李代數 (so(3))，使用 GTSAM 的 Logmap
            Eigen::Vector3d so3 = gtsam::Rot3::Logmap(SO3);
            // 提取旋轉軸（單位向量）
            Eigen::Vector3d a = so3.normalized();
            gtsam::Matrix3 a_hat = gtsam::skewSymmetric(a);
            // 提取旋轉角度（弧度）
            double angle = so3.norm();

            gtsam::Matrix3 J = (std::sin(angle) / angle) * Eigen::Matrix3d::Identity() +
                        ((1 - std::sin(angle) / angle) * (a * a.transpose())) +
                        ((1 - std::cos(angle)) / angle) * a_hat;
            
            H_right.block<3, 3>(0, 0).setZero();  // 空矩陣 0_{3x3}
            H_right.block<3, 3>(0, 3) = -skew_RWGk;  // 上三行
            H_right.block<1, 3>(3, 0) = -(R_k_W * G_k).transpose() * J;  // 1x3 負的轉置
            H_right.block<1, 3>(3, 3) = (pose.translation().transpose() * skew_RWGk);  // 1x3
            

            // 最終的雅可比矩陣是兩個矩陣的乘積
            *H = H_left * H_right;
        }

        // 將兩個誤差結合成一個
        gtsam::Vector weightedError(3);  // 假設殘差是 3 維
        weightedError = noiseModel_->whiten(error);
        


        return weightedError;
    }
};

#endif // GROUNDPLANEFACTOR_H
