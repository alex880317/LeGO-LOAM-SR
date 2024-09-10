#ifndef GROUNDPLANEFACTOR_H
#define GROUNDPLANEFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h> // 引入GTSAM非線性因子基類
#include <gtsam/geometry/Pose3.h>            // 引入Pose3幾何類
#include <gtsam/geometry/Point3.h>           // 引入Point3幾何類
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>

class GroundPlaneFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
    gtsam::Vector3 measuredNormal_; // 地面法向量測量
    double measuredDistance_;       // 地面距離測量

    // 合併的噪聲模型
    gtsam::SharedNoiseModel noiseModel_;

    GroundPlaneFactor(gtsam::Key key, const gtsam::Point3 &normal, const double &distance,
                      const gtsam::SharedNoiseModel &noiseModel)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noiseModel, key),
          measuredNormal_(normal), measuredDistance_(distance),
          noiseModel_(noiseModel) {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
        // 計算距離誤差
        double initialDistance = 0;

        // 計算法向量誤差
        gtsam::Vector3 initialNormal(0.0, 0.0, 1.0);

        gtsam::Matrix3 R_k_W = pose.rotation().matrix();
        gtsam::Vector3 measuredNormal_W = R_k_W * measuredNormal_;

        // 計算法向量參數化 \(\tau(G^W_k)\)
        double theta = std::atan2(measuredNormal_W.y(), measuredNormal_W.x());                      // 方位角
        double phi = std::atan2(measuredNormal_W.z(), measuredNormal_W.head<3>().norm());           // 俯仰角
        double d_k_prime = measuredDistance_ - (pose.translation().transpose() * measuredNormal_W); // 直接使用測量的距離值

        gtsam::Vector3 tau_measured(theta, phi, d_k_prime); // 參數化後的測量值

        // 將預測的法向量進行參數化
        double initial_theta = std::atan2(initialNormal.y(), initialNormal.x());
        double initial_phi = std::atan2(initialNormal.z(), initialNormal.head<3>().norm());
        double initial_d_k_prime = initialDistance;

        gtsam::Vector3 tau_initial(initial_theta, initial_phi, initial_d_k_prime); // 參數化後的預測值

        gtsam::Vector3 error = tau_measured - tau_initial;

        // 如果需要雅可比矩陣 H，則計算
        if (H)
        {
            H->setZero(3, 6); // Jacobian 大小是 3x6

            gtsam::Vector3 G_k = measuredNormal_;
            gtsam::Vector3 G_k_W = measuredNormal_W; // 假設地面法向量是 G_k_W

            // 構建雅可比矩陣
            gtsam::Matrix H_left(3, 4);
            H_left << -G_k_W(1) / (G_k_W(0) * G_k_W(0) + G_k_W(1) * G_k_W(1)), G_k_W(0) / (G_k_W(0) * G_k_W(0) + G_k_W(1) * G_k_W(1)), 0, 0,
                -G_k_W(0) * G_k_W(2) / G_k_W.norm(), -G_k_W(1) * G_k_W(2) / G_k_W.norm(), (G_k_W(0) * G_k_W(0) + G_k_W(1) * G_k_W(1)) / G_k_W.norm(), 0,
                0, 0, 0, 1;

            // 使用反對稱矩陣構建旋轉的雅可比
            gtsam::Matrix H_right(4, 6);
            gtsam::Matrix3 skew_RWGk = gtsam::skewSymmetric(R_k_W * G_k);

            // gtsam::Rot3 SO3 = gtsam::Rot3(R_k_W);
            // // 將李群轉換為李代數 (so(3))，使用 GTSAM 的 Logmap
            // Eigen::Vector3d so3 = gtsam::Rot3::Logmap(SO3);
            // 使用 GTSAM 的 Logmap 函數將 Pose3 轉換為小 se(3)
            Eigen::Matrix<double, 6, 1> se3 = gtsam::Pose3::Logmap(pose);
            Eigen::Vector3d so3 = se3.tail<3>();
            Eigen::Vector3d rho = se3.head<3>();
            // 提取旋轉軸（單位向量）
            Eigen::Vector3d a = so3.normalized();
            gtsam::Matrix3 a_hat = gtsam::skewSymmetric(a);
            // 提取旋轉角度（弧度）
            double angle = so3.norm();

            gtsam::Matrix3 J = (std::sin(angle) / angle) * Eigen::Matrix3d::Identity() +
                               ((1 - std::sin(angle) / angle) * (a * a.transpose())) +
                               ((1 - std::cos(angle)) / angle) * a_hat;

            std::vector<double> sig_phi1 = calculate_sigma_phi1(rho[0], rho[1], rho[2], so3[0], so3[1], so3[2]);
            std::vector<double> sig_phi2 = calculate_sigma_phi2(rho[0], rho[1], rho[2], so3[0], so3[1], so3[2]);
            std::vector<double> sig_phi3 = calculate_sigma_phi3(rho[0], rho[1], rho[2], so3[0], so3[1], so3[2]);
            gtsam::Matrix3 J_rho_diff;
            J_rho_diff << sig_phi1[0], sig_phi2[0], sig_phi3[0],
                sig_phi1[1], sig_phi2[1], sig_phi3[1],
                sig_phi1[2], sig_phi2[2], sig_phi3[2];

            // H_right.block<3, 3>(0, 0).setZero();                                                                                  // 空矩陣 0_{3x3}
            // H_right.block<3, 3>(0, 3) = -skew_RWGk;                                                                     // 上三行
            // H_right.block<1, 3>(3, 0) = -(R_k_W * G_k).transpose() * J;                                                           // 1x3 負的轉置
            // H_right.block<1, 3>(3, 3) = -(J_rho_diff * (R_k_W * G_k)).transpose() + (pose.translation().transpose() * skew_RWGk); // 1x3          
            H_right.block<3, 3>(0, 0).setZero();                                                                                  // 空矩陣 0_{3x3}
            H_right.block<3, 3>(0, 3).setZero();                                                                               // 上三行
            H_right.block<1, 3>(3, 0) = -(R_k_W * G_k).transpose() * J;                                                           // 1x3 負的轉置
            H_right.block<1, 3>(3, 3).setZero(); // 1x3

            // 最終的雅可比矩陣是兩個矩陣的乘積
            *H = H_left * H_right;
        }

        // 將兩個誤差結合成一個
        gtsam::Vector weightedError(3); // 假設殘差是 3 維
        weightedError = noiseModel_->whiten(error);

        return weightedError;
    }

private:
    // 輔助函數：計算 sigma_phi1 向量結果
    std::vector<double> calculate_sigma_phi1(double rho1, double rho2, double rho3, double phi1, double phi2, double phi3) const
    {
        std::vector<double> result(3);

        const double epsilon = 1e-10;  // 防止數值不穩定的 epsilon

        // 計算 sigma_17
        double sigma17 = pow(abs(phi1), 2) + pow(abs(phi2), 2) + pow(abs(phi3), 2);
        double safe_sigma17 = std::max(sigma17, epsilon);

        // 計算 sigma_1 到 sigma_16
        double sigma13 = cos(sqrt(safe_sigma17)) - 1;
        double sigma14 = (sin(sqrt(safe_sigma17)) / sqrt(safe_sigma17)) - 1;

        double sigma1 = (2 * phi3 * abs(phi1) * std::copysign(1.0, phi1) * sigma13) / pow(safe_sigma17, 2);
        double sigma2 = (2 * phi2 * abs(phi1) * std::copysign(1.0, phi1) * sigma13) / pow(safe_sigma17, 2);
        double sigma3 = (2 * phi1 * abs(phi1) * std::copysign(1.0, phi1) * sigma13) / pow(safe_sigma17, 2);
        double sigma4 = (phi3 * sin(sqrt(safe_sigma17)) * abs(phi1) * std::copysign(1.0, phi1)) / pow(safe_sigma17, 1.5);
        double sigma5 = (phi2 * sin(sqrt(safe_sigma17)) * abs(phi1) * std::copysign(1.0, phi1)) / pow(safe_sigma17, 1.5);
        double sigma6 = (phi1 * sin(sqrt(safe_sigma17)) * abs(phi1) * std::copysign(1.0, phi1)) / pow(safe_sigma17, 1.5);
        double sigma7 = (2 * phi2 * phi3 * abs(phi1) * std::copysign(1.0, phi1) * sigma14) / pow(safe_sigma17, 2);
        double sigma8 = (2 * phi1 * phi3 * abs(phi1) * std::copysign(1.0, phi1) * sigma14) / pow(safe_sigma17, 2);
        double sigma9 = (2 * phi1 * phi2 * abs(phi1) * std::copysign(1.0, phi1) * sigma14) / pow(safe_sigma17, 2);
        double sigma15 = (sin(sqrt(safe_sigma17)) * abs(phi1) * std::copysign(1.0, phi1)) / pow(safe_sigma17, 1.5);
        double sigma16 = (cos(sqrt(safe_sigma17)) * abs(phi1) * std::copysign(1.0, phi1)) / safe_sigma17;
        double sigma10 = (phi2 * phi3 * (sigma16 - sigma15)) / safe_sigma17;
        double sigma11 = (phi1 * phi3 * (sigma16 - sigma15)) / safe_sigma17;
        double sigma12 = (phi1 * phi2 * (sigma16 - sigma15)) / safe_sigma17;
        

        // 計算向量結果
        result[0] = rho3 * (sigma5 - sigma11 - (phi3 * sigma14) / safe_sigma17 + sigma2 + sigma8) - rho1 * ((pow(phi1, 2) * (sigma16 - sigma15)) / safe_sigma17 + (2 * phi1 * sigma14) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi1, 2) * abs(phi1) * std::copysign(1.0, phi1) * sigma14) / pow(safe_sigma17, 2)) - rho2 * ((phi2 * sigma14) / safe_sigma17 - sigma13 / safe_sigma17 + sigma12 + sigma6 + sigma3 - sigma9);

        result[1] = -rho3 * (sigma10 + sigma4 + sigma1 - sigma7) - rho2 * ((pow(phi2, 2) * (sigma16 - sigma15)) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi2, 2) * abs(phi1) * std::copysign(1.0, phi1) * sigma14) / pow(safe_sigma17, 2)) - rho1 * (sigma13 / safe_sigma17 + (phi2 * sigma14) / safe_sigma17 + sigma12 - sigma6 - sigma3 - sigma9);

        result[2] = rho2 * (sigma4 - sigma10 + sigma1 + sigma7) - rho1 * ((phi3 * sigma14) / safe_sigma17 + sigma11 + sigma5 + sigma2 - sigma8) - rho3 * ((pow(phi3, 2) * (sigma16 - sigma15)) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi3, 2) * abs(phi1) * std::copysign(1.0, phi1) * sigma14) / pow(safe_sigma17, 2));

        return result;
    }

    // 輔助函數：計算向量結果
    std::vector<double> calculate_sigma_phi2 (double rho1, double rho2, double rho3, double phi1, double phi2, double phi3) const
    {
        std::vector<double> result(3);

        const double epsilon = 1e-10;  // 防止數值不穩定的 epsilon

        // 計算 sigma_17
        double sigma17 = pow(abs(phi1), 2) + pow(abs(phi2), 2) + pow(abs(phi3), 2);
        double safe_sigma17 = std::max(sigma17, epsilon);

        // 計算 sigma_1 到 sigma_16
        double sigma13 = cos(sqrt(safe_sigma17)) - 1;
        double sigma14 = (sin(sqrt(safe_sigma17)) / sqrt(safe_sigma17)) - 1;

        double sigma1 = (2 * phi3 * abs(phi2) * std::copysign(1.0, phi2) * sigma13) / pow(safe_sigma17, 2);
        double sigma2 = (2 * phi2 * abs(phi2) * std::copysign(1.0, phi2) * sigma13) / pow(safe_sigma17, 2);
        double sigma3 = (2 * phi1 * abs(phi2) * std::copysign(1.0, phi2) * sigma13) / pow(safe_sigma17, 2);
        double sigma4 = (phi3 * sin(sqrt(safe_sigma17)) * abs(phi2) * std::copysign(1.0, phi2)) / pow(safe_sigma17, 1.5);
        double sigma5 = (phi2 * sin(sqrt(safe_sigma17)) * abs(phi2) * std::copysign(1.0, phi2)) / pow(safe_sigma17, 1.5);
        double sigma6 = (phi1 * sin(sqrt(safe_sigma17)) * abs(phi2) * std::copysign(1.0, phi2)) / pow(safe_sigma17, 1.5);
        double sigma7 = (2 * phi2 * phi3 * abs(phi2) * std::copysign(1.0, phi2) * sigma14) / pow(safe_sigma17, 2);
        double sigma8 = (2 * phi1 * phi3 * abs(phi2) * std::copysign(1.0, phi2) * sigma14) / pow(safe_sigma17, 2);
        double sigma9 = (2 * phi1 * phi2 * abs(phi2) * std::copysign(1.0, phi2) * sigma14) / pow(safe_sigma17, 2);
        double sigma15 = (sin(sqrt(safe_sigma17)) * abs(phi2) * std::copysign(1.0, phi2)) / pow(safe_sigma17, 1.5);
        double sigma16 = (cos(sqrt(safe_sigma17)) * abs(phi2) * std::copysign(1.0, phi2)) / safe_sigma17;
        double sigma10 = (phi2 * phi3 * (sigma16 - sigma15)) / safe_sigma17;
        double sigma11 = (phi1 * phi3 * (sigma16 - sigma15)) / safe_sigma17;
        double sigma12 = (phi1 * phi2 * (sigma16 - sigma15)) / safe_sigma17;
        

        // 計算向量結果
        result[0] = rho3 * (sigma5 - sigma11 - sigma13 / safe_sigma17 + sigma2 + sigma8) - rho1 * (pow(phi1, 2) * (sigma16 - sigma15) / safe_sigma17 - sigma16 + sigma15 - 2 * pow(phi1, 2) * abs(phi2) * std::copysign(1.0, phi2) * sigma14 / pow(safe_sigma17, 2)) - rho2 * ((phi1 * sigma14) / safe_sigma17 + sigma12 + sigma6 + sigma3 - sigma9);

        result[1] = rho1 * (sigma6 - sigma12 - (phi1 * sigma14) / safe_sigma17 + sigma3 + sigma9) - rho3 * (phi3 * sigma14 / safe_sigma17 + sigma10 + sigma4 + sigma1 - sigma7) - rho2 * (pow(phi2, 2) * (sigma16 - sigma15) / safe_sigma17 + 2 * phi2 * sigma14 / safe_sigma17 - sigma16 + sigma15 - 2 * pow(phi2, 2) * abs(phi2) * std::copysign(1.0, phi2) * sigma14 / pow(safe_sigma17, 2));

        result[2] = rho2 * (sigma4 - sigma10 - (phi3 * sigma14) / safe_sigma17 + sigma1 + sigma7) - rho3 * (pow(phi3, 2) * (sigma16 - sigma15) / safe_sigma17 - sigma16 + sigma15 - 2 * pow(phi3, 2) * abs(phi2) * std::copysign(1.0, phi2) * sigma14 / pow(safe_sigma17, 2)) - rho1 * (sigma11 - sigma13 / safe_sigma17 + sigma5 + sigma2 - sigma8);

        return result;
    }

    // 輔助函數：計算 sigma_phi3 向量結果
    std::vector<double> calculate_sigma_phi3 (double rho1, double rho2, double rho3, double phi1, double phi2, double phi3) const
    {
        std::vector<double> result(3);

        const double epsilon = 1e-10;  // 防止數值不穩定的 epsilon

        // 計算 sigma_17
        double sigma17 = pow(abs(phi1), 2) + pow(abs(phi2), 2) + pow(abs(phi3), 2);
        double safe_sigma17 = std::max(sigma17, epsilon);

        // 計算 sigma_13 到 sigma_16
        double sigma13 = cos(sqrt(safe_sigma17)) - 1;
        double sigma14 = (sin(sqrt(safe_sigma17)) / sqrt(safe_sigma17)) - 1;

        double sigma1 = (2 * phi3 * abs(phi3) * std::copysign(1.0, phi3) * sigma13) / pow(safe_sigma17, 2);
        double sigma2 = (2 * phi2 * abs(phi3) * std::copysign(1.0, phi3) * sigma13) / pow(safe_sigma17, 2);
        double sigma3 = (2 * phi1 * abs(phi3) * std::copysign(1.0, phi3) * sigma13) / pow(safe_sigma17, 2);
        double sigma4 = (phi3 * sin(sqrt(safe_sigma17)) * abs(phi3) * std::copysign(1.0, phi3)) / pow(safe_sigma17, 1.5);
        double sigma5 = (phi2 * sin(sqrt(safe_sigma17)) * abs(phi3) * std::copysign(1.0, phi3)) / pow(safe_sigma17, 1.5);
        double sigma6 = (phi1 * sin(sqrt(safe_sigma17)) * abs(phi3) * std::copysign(1.0, phi3)) / pow(safe_sigma17, 1.5);
        double sigma7 = (2 * phi2 * phi3 * abs(phi3) * std::copysign(1.0, phi3) * sigma14) / pow(safe_sigma17, 2);
        double sigma8 = (2 * phi1 * phi3 * abs(phi3) * std::copysign(1.0, phi3) * sigma14) / pow(safe_sigma17, 2);
        double sigma9 = (2 * phi1 * phi2 * abs(phi3) * std::copysign(1.0, phi3) * sigma14) / pow(safe_sigma17, 2);
        double sigma15 = (sin(sqrt(safe_sigma17)) * abs(phi3) * std::copysign(1.0, phi3)) / pow(safe_sigma17, 1.5);
        double sigma16 = (cos(sqrt(safe_sigma17)) * abs(phi3) * std::copysign(1.0, phi3)) / safe_sigma17;
        double sigma10 = (phi2 * phi3 * (sigma16 - sigma15)) / safe_sigma17;
        double sigma11 = (phi1 * phi3 * (sigma16 - sigma15)) / safe_sigma17;
        double sigma12 = (phi1 * phi2 * (sigma16 - sigma15)) / safe_sigma17;
        

        // 計算向量結果
        result[0] = rho3 * (sigma5 - sigma11 - (phi1 * sigma14) / safe_sigma17 + sigma2 + sigma8) - rho2 * (sigma12 + sigma6 + sigma3 - sigma9) - rho1 * (pow(phi1, 2) * (sigma16 - sigma15) / safe_sigma17 - sigma16 + sigma15 - 2 * pow(phi1, 2) * abs(phi3) * std::copysign(1.0, phi3) * sigma14 / pow(safe_sigma17, 2));

        result[1] = rho1 * (sigma6 - sigma12 + sigma3 + sigma9) - rho2 * (pow(phi2, 2) * (sigma16 - sigma15) / safe_sigma17 - sigma16 + sigma15 - 2 * pow(phi2, 2) * abs(phi3) * std::copysign(1.0, phi3) * sigma14 / pow(safe_sigma17, 2)) - rho3 * (phi2 * sigma14 / safe_sigma17 - sigma13 / safe_sigma17 + sigma10 + sigma4 + sigma1 - sigma7);

        result[2] = -rho1 * (phi1 * sigma14 / safe_sigma17 + sigma11 + sigma5 + sigma2 - sigma8) - rho3 * (pow(phi3, 2) * (sigma16 - sigma15) / safe_sigma17 + 2 * phi3 * sigma14 / safe_sigma17 - sigma16 + sigma15 - 2 * pow(phi3, 2) * abs(phi3) * std::copysign(1.0, phi3) * sigma14 / pow(safe_sigma17, 2)) - rho2 * (sigma13 / safe_sigma17 + phi2 * sigma14 / safe_sigma17 + sigma10 - sigma4 - sigma1 - sigma7);

        return result;
    }
};

#endif // GROUNDPLANEFACTOR_H
