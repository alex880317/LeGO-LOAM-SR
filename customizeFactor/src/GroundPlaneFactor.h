#ifndef GROUNDPLANEFACTOR_H
#define GROUNDPLANEFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h> // 引入GTSAM非線性因子基類
#include <gtsam/geometry/Pose3.h>            // 引入Pose3幾何類
#include <gtsam/geometry/Point3.h>           // 引入Point3幾何類
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sstream> // for std::stringstream

class GroundPlaneFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
private:
    gtsam::Vector3 measuredNormal_;      // 地面法向量測量
    double measuredDistance_;            // 地面距離測量
    const gtsam::Vector3 G_k;            // 單位法向量
    gtsam::SharedNoiseModel noiseModel_; // 噪声模型

    bool isActive_; // 新增的布爾型成員變量

public:
    using NoiseModelFactor1<gtsam::Pose3>::evaluateError;

    typedef std::shared_ptr<GroundPlaneFactor> shared_ptr;

    GroundPlaneFactor(gtsam::Key key, const gtsam::Point3 &normal, const double &distance,
                      const gtsam::SharedNoiseModel &noiseModel, rclcpp::Node::SharedPtr node,
                      bool isActive = true)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noiseModel, key),
          measuredNormal_(normal),
          measuredDistance_(distance),
          G_k(normal.normalized()), // 在構造函數中初始化 G_k_
          noiseModel_(noiseModel),
          node_(node),
          isActive_(isActive) // 初始化布爾型變量
    {
        // RCLCPP_INFO(node_->get_logger(), "out : Time: %.6f, G_k = [%.6f, %.6f, %.6f], measuredNormal_ = [%.6f, %.6f, %.6f]", node_->now().seconds(),
        // G_k(0), G_k(1), G_k(2), measuredNormal_(0), measuredNormal_(1), measuredNormal_(2));
    }

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {

        double initialDistance = 1.78; // Gazebo:0.12 Mulran:1.78 Carla:2.46 // d0

        // 計算法向量誤差
        gtsam::Vector3 initialNormal(0.0, 0.0, 1.0);

        // gtsam::Pose3 p_inv = pose.inverse();
        gtsam::Matrix3 R_k_W = pose.rotation().matrix();
        gtsam::Vector3 t_k_W = pose.translation();


        // RCLCPP_INFO(node_->get_logger(), "up : Time: %.6f, Key: %lu, G_k = [%.6f, %.6f, %.6f], measuredNormal_ = [%.6f, %.6f, %.6f]", node_->now().seconds(), this->key(),
        //     G_k(0), G_k(1), G_k(2), measuredNormal_(0), measuredNormal_(1), measuredNormal_(2));

        gtsam::Vector3 measuredNormal_W = R_k_W * G_k;

        // std::cout << "G_k = " << G_k << std::endl;

        // 計算法向量參數化 \(\tau(G^W_k)\)
        double theta = std::atan2(measuredNormal_W.y(), measuredNormal_W.x());            // 方位角
        double phi = std::acos(measuredNormal_W.z() / measuredNormal_W.head<3>().norm()); // 俯仰角
        double inner_product_term = (t_k_W.transpose() * measuredNormal_W);
        double d_k_prime = (measuredDistance_ + inner_product_term);  // 直接使用測量的距離值

        gtsam::Vector3 tau_measured(theta, phi, d_k_prime); // 參數化後的測量值

        // 將預測的法向量進行參數化
        double initial_theta = std::atan2(initialNormal.y(), initialNormal.x());
        double initial_phi = std::acos(initialNormal.z() / initialNormal.head<3>().norm());
        double initial_d_k_prime = initialDistance;

        gtsam::Vector3 tau_initial(initial_theta, initial_phi, initial_d_k_prime);

        gtsam::Vector3 error = tau_measured - tau_initial;

        // std::cout << "residual = " << error << std::endl;

        gtsam::Vector3 G_k_W = measuredNormal_W;

        

        // 如果需要雅可比矩陣 H，則計算
        if (H)
        {
            H->setZero(1, 6); // Jacobian 大小是 3x6

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            Eigen::Vector3d H11_left;
            Eigen::Vector3d H21_left;

            // 計算 denominator
            double sqrt_term = std::sqrt(1 - (G_k_W(2) * G_k_W(2)) / (G_k_W.norm() * G_k_W.norm()));
            double denominator = pow(G_k_W.norm(), 3.0) * sqrt_term;
            // 使用反對稱矩陣構建旋轉的雅可比
            gtsam::Matrix3 skew_RWGk = R_k_W * gtsam::skewSymmetric(G_k);

            H11_left << -G_k_W(1) / (G_k_W(0) * G_k_W(0) + G_k_W(1) * G_k_W(1)), 
                        G_k_W(0) / (G_k_W(0) * G_k_W(0) + G_k_W(1) * G_k_W(1)), 
                        0.0;
            H21_left << (G_k_W(2) * G_k_W(0)) / denominator, 
                        (G_k_W(2) * G_k_W(1)) / denominator, 
                        -((G_k_W(0) * G_k_W(0) + G_k_W(1) * G_k_W(1))) / denominator;
            // H21_left << - (G_k_W.squaredNorm() * G_k_W(2) * G_k_W(0)) / (pow(G_k_W.norm(), 3) * (G_k_W.squaredNorm() + pow(G_k_W(2), 2))),
            //             - (G_k_W.squaredNorm() * G_k_W(2) * G_k_W(1)) / (pow(G_k_W.norm(), 3) * (G_k_W.squaredNorm() + pow(G_k_W(2), 2))),
            //             (G_k_W.squaredNorm() * (G_k_W.squaredNorm() - pow(G_k_W(2), 2))) / (pow(G_k_W.norm(), 3) * (G_k_W.squaredNorm() + pow(G_k_W(2), 2)));
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // 使用 GTSAM 的 Logmap 函數將 Pose3 轉換為小 se(3)
            Eigen::Matrix<double, 6, 1> se3 = gtsam::Pose3::Logmap(pose);
            Eigen::Vector3d so3 = se3.head<3>();
            Eigen::Vector3d rho = se3.tail<3>(); // question what is the order of se3 in gtsam????
            // 提取旋轉軸（單位向量）
            Eigen::Vector3d a = so3.normalized();
            gtsam::Matrix3 a_hat = gtsam::skewSymmetric(a);
            // 提取旋轉角度（弧度）
            double angle = so3.norm();
            // std::cout << "angle = " << angle << std::endl;
            
            // Right Perturbation Jacobian
            gtsam::Matrix3 J = (std::sin(angle) / angle) * Eigen::Matrix3d::Identity() +
                               ((1 - std::sin(angle) / angle) * (a * a.transpose())) -
                               ((1 - std::cos(angle)) / angle) * a_hat;
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // std::vector<double> sig_phi1 = calculate_sigma_phi1(rho[0], rho[1], rho[2], so3[0], so3[1], so3[2]);
            // std::vector<double> sig_phi2 = calculate_sigma_phi2(rho[0], rho[1], rho[2], so3[0], so3[1], so3[2]);
            // std::vector<double> sig_phi3 = calculate_sigma_phi3(rho[0], rho[1], rho[2], so3[0], so3[1], so3[2]);
            // gtsam::Matrix3 J_rho_diff;
            // J_rho_diff << sig_phi1[0], sig_phi2[0], sig_phi3[0],
            //     sig_phi1[1], sig_phi2[1], sig_phi3[1],
            //     sig_phi1[2], sig_phi2[2], sig_phi3[2];

            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // gtsam::Matrix3 dGdEulerAngles = computeEulerAngleDerivatives(R_k_W, G_k);
            // gtsam::Matrix dPdotGdEulerAngles = computePdotEulerAngleDerivatives(R_k_W, G_k, t_k_W);
            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

            Eigen::MatrixXd& H_matrix = *H;  // 解包 boost::optional
            // H_matrix.block<1, 3>(0, 3) = H11_left.transpose() * (dGdEulerAngles);
            // H_matrix.block<1, 3>(1, 3) = H21_left.transpose() * (dGdEulerAngles);
            // H_matrix.block<1, 3>(2, 3) = dPdotGdEulerAngles;     
            // H_matrix.block<1, 3>(0, 0).setZero();
            // H_matrix.block<1, 3>(1, 0).setZero();
            // H_matrix.block<1, 3>(2, 0) = (R_k_W * G_k);

            // H_matrix.block<1, 3>(0, 0) = H11_left.transpose() * (-skew_RWGk);
            // H_matrix.block<1, 3>(1, 0) = H21_left.transpose() * (-skew_RWGk);
            H_matrix.block<1, 3>(0, 0) =  - (t_k_W.transpose() * skew_RWGk);   // (J_rho_diff.transpose() * (R_k_W * G_k)).transpose()
            // H_matrix.block<1, 3>(0, 3).setZero();
            // H_matrix.block<1, 3>(1, 3).setZero();
            H_matrix.block<1, 3>(0, 3) = (J.transpose() * (R_k_W * G_k)).transpose();

            

            // if (isActive_){
            //     // // 打印 H_left
            //     // {
            //     //     std::stringstream ss_left;
            //     //     ss_left << H_left.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", "\n", "[", "]"));
            //     //     RCLCPP_INFO(node_->get_logger(), "Jacobian H_left:\n%s", ss_left.str().c_str());
            //     // }

            //     // // 打印 H_right
            //     // {
            //     //     std::stringstream ss_right;
            //     //     ss_right << H_right.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", "\n", "[", "]"));
            //     //     RCLCPP_INFO(node_->get_logger(), "Jacobian H_right:\n%s", ss_right.str().c_str());
            //     // }

            //     // 打印 H
            //     {
            //         std::stringstream ss_total;
            //         ss_total << H_matrix.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", "\n", "[", "]"));
            //         RCLCPP_INFO(node_->get_logger(), "Jacobian H_total:\n%s", ss_total.str().c_str());
            //     }

            //     RCLCPP_INFO(node_->get_logger(), "T_k in Jacobian : [%f, %f, %f]", t_k_W(0), t_k_W(1), t_k_W(2));
            //     // 打印 skew_RWGk
            //     {
            //         std::stringstream ss_skew_RWGk;
            //         ss_skew_RWGk << skew_RWGk.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", "\n", "[", "]"));
            //         RCLCPP_INFO(node_->get_logger(), "Jacobian skew_RWGk:\n%s", ss_skew_RWGk.str().c_str());
            //     }
            // }
        }
        // error[0] = 0;
        // 將兩個誤差結合成一個
        // gtsam::Vector weightedError(3); // 假設殘差是 3 維
        // weightedError = noiseModel_->whiten(error);

        // if (isActive_){
        //     Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

        //     std::stringstream ss;
        //     ss << error.transpose().format(CleanFmt);
        //     RCLCPP_INFO(node_->get_logger(), "Time: %f, error = %s", node_->now().seconds(), ss.str().c_str());
        //     // std::cout << "Time: " << node_->now().seconds() << ", error = " << ss.str() << std::endl;
        //     // RCLCPP_INFO(node_->get_logger(), "Time: %.6f, G_k = [%.6f, %.6f, %.6f], measuredNormal_ = [%.6f, %.6f, %.6f], error = [%.6f, %.6f, %.6f]", node_->now().seconds(),
        //     //     G_k_W(0), G_k_W(1), G_k_W(2), measuredNormal_(0), measuredNormal_(1), measuredNormal_(2), error[0], error[1], error[2]);

        //     double cost = std::pow(error[0], 2) + std::pow(error[1], 2) + std::pow(error[2], 2);
        //     RCLCPP_INFO(node_->get_logger(), "Cost = %f", cost);
        //     // std::cout << "Cost = " << cost << std::endl;

        //     gtsam::Vector3 rot = calculateZYXEulerAngles(R_k_W);
        //     RCLCPP_INFO(node_->get_logger(), "eular angle (Body frame with respect to World frame) (Factor) : [%f, %f, %f]", rot(2), rot(1), rot(0));
        //     RCLCPP_INFO(node_->get_logger(), "translation (Body frame with respect to World frame) (Factor) : [%f, %f, %f]", t_k_W(0), t_k_W(1), t_k_W(2));
            
        //     RCLCPP_INFO(node_->get_logger(), "tau_measured is: [%f, %f, %f]", tau_measured(0), tau_measured(1), tau_measured(2));
        //     RCLCPP_INFO(node_->get_logger(), "tau_initial is: [%f, %f, %f]", tau_initial(0), tau_initial(1), tau_initial(2));
        //     RCLCPP_INFO(node_->get_logger(), "measuredNormal_W: [%f, %f, %f]",
        //         measuredNormal_W(0), measuredNormal_W(1), measuredNormal_W(2));
            
        //     // double inner_product_term = (t_k_W.transpose() * measuredNormal_W);
        //     RCLCPP_INFO(node_->get_logger(), "The inner product term is: %f", inner_product_term);
        //     RCLCPP_INFO(node_->get_logger(), "The measuredDistance is: %f", measuredDistance_);
            
        // }
        

        // std::cout << "weightedError = " << weightedError.transpose() << std::endl;

        gtsam::Vector error_star = error.tail<1>();

        // return weightedError;
        return error_star;
        // return error;
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new GroundPlaneFactor(*this)));
    }

    // 成員函數：設置布爾型變量的值
    void setBool(bool value)
    {
        isActive_ = value;
    }

    // 成員函數：獲取布爾型變量的值
    bool getBool() const
    {
        return isActive_;
    }

private:
    rclcpp::Node::SharedPtr node_;

    // 輔助函數：計算 sigma_phi1 向量結果
    std::vector<double> calculate_sigma_phi1(double rho1, double rho2, double rho3, double phi1, double phi2, double phi3) const
    {
        std::vector<double> result(3);

        const double epsilon = 1e-10; // 防止數值不穩定的 epsilon

        // 計算 sigma_17
        double sigma17 = pow(abs(phi1), 2) + pow(abs(phi2), 2) + pow(abs(phi3), 2);
        double safe_sigma17 = std::max(sigma17, epsilon); // 用 epsilon 防止 sigma17 為 0

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

        // 計算向量結果，根據您提供的正確公式進行更改
        result[0] = rho3 * (sigma5 - sigma11 - (phi3 * sigma14) / safe_sigma17 + sigma2 + sigma8) - rho2 * ((phi2 * sigma14) / safe_sigma17 + sigma12 + sigma4 + sigma1 - sigma9) - rho1 * ((pow(phi1, 2) * (sigma16 - sigma15)) / safe_sigma17 + (2 * phi1 * sigma14) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi1, 2) * abs(phi1) * std::copysign(1.0, phi1) * sigma14) / pow(safe_sigma17, 2));

        result[1] = rho1 * (sigma4 - sigma12 - (phi2 * sigma14) / safe_sigma17 + sigma1 + sigma9) - rho2 * ((pow(phi2, 2) * (sigma16 - sigma15)) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi2, 2) * abs(phi1) * std::copysign(1.0, phi1) * sigma14) / pow(safe_sigma17, 2)) - rho3 * (sigma10 - (sigma13 / safe_sigma17) + sigma6 + sigma3 - sigma7);

        result[2] = rho2 * (sigma6 - sigma10 - (sigma13 / safe_sigma17) + sigma3 + sigma7) - rho3 * ((pow(phi3, 2) * (sigma16 - sigma15)) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi3, 2) * abs(phi1) * std::copysign(1.0, phi1) * sigma14) / pow(safe_sigma17, 2)) - rho1 * ((phi3 * sigma14) / safe_sigma17 + sigma11 + sigma5 + sigma2 - sigma8);

        return result;
    }

    // 輔助函數：計算向量結果
    std::vector<double> calculate_sigma_phi2(double rho1, double rho2, double rho3, double phi1, double phi2, double phi3) const
    {
        std::vector<double> result(3);

        const double epsilon = 1e-10; // 防止數值不穩定的 epsilon

        // 計算 sigma_17
        double sigma17 = pow(abs(phi1), 2) + pow(abs(phi2), 2) + pow(abs(phi3), 2);
        double safe_sigma17 = std::max(sigma17, epsilon); // 用 epsilon 防止 sigma17 為 0

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

        // 計算向量結果，根據您提供的正確公式進行更改
        result[0] = rho3 * (sigma5 - sigma11 - (sigma13 / safe_sigma17) + sigma2 + sigma8) - rho1 * ((pow(phi1, 2) * (sigma16 - sigma15)) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi1, 2) * abs(phi2) * std::copysign(1.0, phi2) * sigma14) / pow(safe_sigma17, 2)) - rho2 * ((phi1 * sigma14) / safe_sigma17 + sigma12 + sigma4 + sigma1 - sigma9);

        result[1] = rho1 * (sigma4 - sigma12 - (phi1 * sigma14) / safe_sigma17 + sigma1 + sigma9) - rho3 * ((phi3 * sigma14) / safe_sigma17 + sigma10 + sigma6 + sigma3 - sigma7) - rho2 * ((pow(phi2, 2) * (sigma16 - sigma15)) / safe_sigma17 + (2 * phi2 * sigma14) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi2, 2) * abs(phi2) * std::copysign(1.0, phi2) * sigma14) / pow(safe_sigma17, 2));

        result[2] = rho2 * (sigma6 - sigma10 - (phi3 * sigma14) / safe_sigma17 + sigma3 + sigma7) - rho3 * ((pow(phi3, 2) * (sigma16 - sigma15)) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi3, 2) * abs(phi2) * std::copysign(1.0, phi2) * sigma14) / pow(safe_sigma17, 2)) - rho1 * (sigma11 - sigma13 / safe_sigma17 + sigma5 + sigma2 - sigma8);

        return result;
    }

    // 輔助函數：計算 sigma_phi3 向量結果
    std::vector<double> calculate_sigma_phi3(double rho1, double rho2, double rho3, double phi1, double phi2, double phi3) const
    {
        std::vector<double> result(3);

        const double epsilon = 1e-10; // 防止數值不穩定的 epsilon

        // 計算 sigma_17
        double sigma17 = pow(abs(phi1), 2) + pow(abs(phi2), 2) + pow(abs(phi3), 2);
        double safe_sigma17 = std::max(sigma17, epsilon); // 防止 sigma17 為 0

        // 計算 sigma_1 到 sigma_16
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

        // 計算向量結果，根據新的公式進行調整
        result[0] = rho3 * (sigma5 - sigma11 - (phi1 * sigma14) / safe_sigma17 + sigma2 + sigma8) - rho1 * (pow(phi1, 2) * (sigma16 - sigma15) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi1, 2) * abs(phi3) * std::copysign(1.0, phi3) * sigma14) / pow(safe_sigma17, 2)) - rho2 * (sigma12 - (sigma13 / safe_sigma17) + sigma4 + sigma1 - sigma9);

        result[1] = rho1 * (sigma4 - sigma12 - (sigma13 / safe_sigma17) + sigma1 + sigma9) - rho2 * (pow(phi2, 2) * (sigma16 - sigma15) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi2, 2) * abs(phi3) * std::copysign(1.0, phi3) * sigma14) / pow(safe_sigma17, 2)) - rho3 * ((phi2 * sigma14) / safe_sigma17 + sigma10 + sigma6 + sigma3 - sigma7);

        result[2] = rho2 * (sigma6 - sigma10 - (phi2 * sigma14) / safe_sigma17 + sigma3 + sigma7) - rho1 * ((phi1 * sigma14) / safe_sigma17 + sigma11 + sigma5 + sigma2 - sigma8) - rho3 * (pow(phi3, 2) * (sigma16 - sigma15) / safe_sigma17 + (2 * phi3 * sigma14) / safe_sigma17 - sigma16 + sigma15 - (2 * pow(phi3, 2) * abs(phi3) * std::copysign(1.0, phi3) * sigma14) / pow(safe_sigma17, 2));

        return result;
    }

    // 新增函數：計算ZYX順序的歐拉角 (Z-Y-X)
    // 假設 R = Rz(γ)*Ry(β)*Rx(α)
    // 回傳值為 [γ, β, α]
    gtsam::Vector3 calculateZYXEulerAngles(const gtsam::Matrix3 &rot) const
    {
        // gtsam::Matrix3 R = rot.matrix();

        // R(2,0) = -sin(β)
        double beta = std::asin(-rot(2,0));

        // γ = atan2(R(1,0), R(0,0))
        double gamma = std::atan2(rot(1,0), rot(0,0));

        // α = atan2(R(2,1), R(2,2))
        double alpha = std::atan2(rot(2,1), rot(2,2));

        return gtsam::Vector3(gamma, beta, alpha);
    }

    // Function to compute the first part of the expression (result1)
    double compute_partialR1_partialDk(const gtsam::Vector3& P, const gtsam::Vector3& G) const
    {
        // Extract components from vectors
        double px = P(0);
        double py = P(1);
        double Gx = G(0);
        double Gy = G(1);
    
        // Compute result1
        double denominator1 = Gx * (Gy * Gy / (Gx * Gx) + 1);
        double term1 = py / denominator1;
        double term2 = Gy * px / (Gx * Gx * (Gy * Gy / (Gx * Gx) + 1));
        double numerator1 = term1 - term2;
        double denominator_final1 = px * px + py * py;
        double result1 = numerator1 / denominator_final1;
    
        return result1;
    }
    
    // Function to compute the second part of the expression (result2)
    double compute_partialR2_partialDk(const gtsam::Vector3& P, const gtsam::Vector3& G) const
    {
        // Extract components from vectors
        double px = P(0);
        double py = P(1);
        double Gx = G(0);
        double Gy = G(1);
        double Gz = G(2);
    
        // Compute \sigma_1
        double sigma1 = std::sqrt(1 - (Gz * Gz) / (Gx * Gx + Gy * Gy + Gz * Gz)) * 
                        std::pow(Gx * Gx + Gy * Gy + Gz * Gz, 1.5);
    
        // Compute result2
        double term3 = Gx * Gz * px / sigma1;
        double term4 = Gy * Gz * py / sigma1;
        double numerator2 = term3 + term4;
        double denominator_final1 = px * px + py * py;
        double result2 = numerator2 / denominator_final1;
    
        return result2;
    }

    // Function to compute partial derivatives of R * G_k with respect to Euler angles
    gtsam::Matrix3 computeEulerAngleDerivatives(const gtsam::Matrix3& R, const gtsam::Point3& G_k) const
    {
        // Compute Euler angles (ZYX order) from the rotation matrix
        double theta = std::asin(-R(2, 0));
        double phi = std::atan2(R(2, 1), R(2, 2));
        double psi = std::atan2(R(1, 0), R(0, 0));

        // Compute trigonometric values
        double c_phi = std::cos(phi), s_phi = std::sin(phi);
        double c_theta = std::cos(theta), s_theta = std::sin(theta);
        double c_psi = std::cos(psi), s_psi = std::sin(psi);

        // Extract elements of G_k
        double x_k = G_k.x(), y_k = G_k.y(), z_k = G_k.z();

        // Initialize output matrix (3x3, for dT/dphi, dT/dtheta, dT/dpsi)
        gtsam::Matrix3 derivatives;

        // Partial derivatives with respect to phi (roll)
        derivatives(0, 0) = -s_phi * c_theta * x_k - (s_phi * s_theta * s_psi + c_phi * c_psi) * y_k - (s_phi * s_theta * c_psi - c_phi * s_psi) * z_k;
        derivatives(1, 0) =  c_phi * c_theta * x_k + (c_phi * s_theta * s_psi - s_phi * c_psi) * y_k + (c_phi * s_theta * c_psi + s_phi * s_psi) * z_k;
        derivatives(2, 0) = 0;

        // Partial derivatives with respect to theta (pitch)
        derivatives(0, 1) = -c_phi * s_theta * x_k + c_phi * c_theta * s_psi * y_k + c_phi * c_theta * c_psi * z_k;
        derivatives(1, 1) = -s_phi * s_theta * x_k + s_phi * c_theta * s_psi * y_k + s_phi * c_theta * c_psi * z_k;
        derivatives(2, 1) = -c_theta * x_k - s_theta * s_psi * y_k - s_theta * c_psi * z_k;

        // Partial derivatives with respect to psi (yaw)
        derivatives(0, 2) = 0 * x_k + (c_phi * c_theta * c_psi - s_phi * s_psi) * y_k + (-c_phi * c_theta * s_psi - s_phi * c_psi) * z_k;
        derivatives(1, 2) = 0 * x_k + (s_phi * c_theta * c_psi + c_phi * s_psi) * y_k + (-s_phi * c_theta * s_psi + c_phi * c_psi) * z_k;
        derivatives(2, 2) = 0 * x_k - s_theta * c_psi * y_k + s_theta * s_psi * z_k;

        return derivatives;
    }

    // Function to compute the new partial derivatives with scaling vector P
    gtsam::Matrix computePdotEulerAngleDerivatives(const gtsam::Matrix3& R, const gtsam::Point3& G_k, const gtsam::Vector3& P) const
    {
        // Compute the original derivatives matrix (3x3)
        gtsam::Matrix originalDerivatives = computeEulerAngleDerivatives(R, G_k);

        // Compute the scaled derivatives
        gtsam::Matrix derivatives(1, 3);
        derivatives(0, 0) = P(0) * originalDerivatives(0, 0) + P(1) * originalDerivatives(1, 0) + P(2) * originalDerivatives(2, 0);
        derivatives(0, 1) = P(0) * originalDerivatives(0, 1) + P(1) * originalDerivatives(1, 1) + P(2) * originalDerivatives(2, 1);
        derivatives(0, 2) = P(0) * originalDerivatives(0, 2) + P(1) * originalDerivatives(1, 2) + P(2) * originalDerivatives(2, 2);

        return derivatives;
    }

};

#endif // GROUNDPLANEFACTOR_H