#include "myESKF.h"
#include "MatrixUtils.h"
// -------------------------------------------------------
#include "myexpm.h"
#include <boost/numeric/ublas/io.hpp> // 用於打印矩陣
// -------------------------------------------------------
#include <Eigen/Geometry> //Supported by JaronGPT
#include <fstream>
#include <iostream>
#include <typeinfo>
#include "sophus/se3.hpp"
#include "MeaCovFromMatlab/MeaCov2C_pkg/MeaCov2C.h"

// #include "matplotplusplus/source/matplot/matplot.h"
// #include <cmath>
// #include "approx_dirac.h"
// #include "rclcpp/rclcpp.hpp"
// using namespace boost::numeric::ublas;

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

ESKF::ESKF(const Eigen::MatrixXd& Omega_m, const Eigen::MatrixXd& Acc_m, const Eigen::MatrixXd& GPS, int iter)
    : Omega_m(Omega_m), Acc_m(Acc_m), GPSData(GPS), Iter(iter) {}

void ESKF::ValueInitialize() {

    // 處理姿態從尤拉角轉成四元數
    Eigen::Quaterniond qua = eulerToQuaternion(att[0][0], att[0][1], att[0][2]);
    // std::cout << "Quaternion: [" << qua.w() << ", " << qua.x() << ", " << qua.y() << ", " << qua.z() << "]" << std::endl;

    for (double i = 0; i <= end_time+0.001; i += sampling_time) {
        t.push_back(i);
    }

    for (double i = 0; i <= end_time+0.001; i += sampling_time_slam) {
        t_slam.push_back(i);
    }

    for (double i = 0; i <= end_time+0.001; i += sampling_time_ack) {
        t_ack.push_back(i);
    }

    i_imu_update = 0;
    i_slam_update = 1;
    i_ack_update = 0;

    gravity << 0, 0, -9.8;

    State.Pos = pos[0];
    State.Vel = vel[0];
    State.Qua = qua;
    State.AccBias = accb[0];
    State.GyroBias = gyrob[0];
    State.Grav = gravity;
    NominalState = State;
    // NominalStatePropagation
    // NominalState.Pos = pos[0];
    // NominalState.Vel = vel[0];
    // NominalState.Qua = qua;
    // NominalState.AccBias = accb[0];
    // NominalState.GyroBias = gyrob[0];
    // NominalState.Grav = gravity;
    NominalPropagation.Pos = pos[0];
    NominalPropagation.Vel = vel[0];
    NominalPropagation.Qua = qua;
    NominalPropagation.AccBias = accb[0];
    NominalPropagation.GyroBias = gyrob[0];
    NominalPropagation.Grav = gravity;

    PosError = Eigen::Vector3d::Zero();
    VelError = Eigen::Vector3d::Zero();
    QuaError = Eigen::Vector3d::Zero();
    AccBiasError = Eigen::Vector3d::Zero();
    GyroBiasError = Eigen::Vector3d::Zero();
    GravError = Eigen::Vector3d::Zero();

    P = Eigen::MatrixXd::Identity(18, 18)*0.001;
    P_pri = P;
    P_fusion = P;

    K1.resize(18, 7);
    K1.setZero();
    K2.resize(18, 6);
    K2.setZero();

    XError.resize(18);
    XError.segment<3>(0) = PosError;
    XError.segment<3>(3) = VelError;
    XError.segment<3>(6) = QuaError;
    XError.segment<3>(9) = AccBiasError;
    XError.segment<3>(12) = GyroBiasError;
    XError.segment<3>(15) = GravError;
    // std::cout << XError << std::endl;

    // ErrorStateTransitionMatrix
    Fx = Eigen::MatrixXd::Identity(18, 18);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
    Fx.block<3,3>(0, 3) = I * sampling_time;
    Fx.block<3,3>(3, 15) = I * sampling_time;
    Fx.block<3,3>(6, 12) = -I * sampling_time;

    Fi.resize(18, 12);
    Fi.setZero();
    Fi.block<3,3>(3, 0) = I;
    Fi.block<3,3>(6, 3) = I;
    Fi.block<3,3>(9, 6) = I;
    Fi.block<3,3>(12, 9) = I;


    Qi = Eigen::MatrixXd::Identity(12, 12);
    IMU_noise_scale.resize(4);
    IMU_noise_scale << AccSTD , GyroSTD , AccBiasSTD , GyroBiasSTD;

    // std::cout << IMU_noise_scale <<std::endl;

    // Measurement1 Initial Value
    Xdeltax.resize(19, 18);
    Xdeltax.setZero();
    Eigen::MatrixXd matrix_18x18 = Eigen::MatrixXd::Identity(18, 18);
    // std::cout << Xdeltax <<std::endl;
    Xdeltax.topRows(6) = matrix_18x18.topRows(6);
    Xdeltax.row(6).setZero();
    // std::cout << Xdeltax <<std::endl;
    Xdeltax.block(7, 0, 11, 18) = matrix_18x18.bottomRows(11);
    // std::cout << Xdeltax <<std::endl;
    // std::cout << matrix_18x18.bottomRows(11) <<std::endl;

    Lidar_noise_scale.resize(2);
    Lidar_noise_scale << TranslationSTD , RotationSTD;
    Hx1(0,0) = 1; Hx1(1,1) = 1; Hx1(2,2) = 1;
    Hx1(3,6) = 1; Hx1(4,7) = 1; Hx1(5,8) = 1; Hx1(6,9) = 1;


    Hx2(0,3) = 1;Hx2(1,4) = 1;Hx2(2,6) = 1;Hx2(3,7) = 1;Hx2(4,8) = 1;Hx2(5,9) = 1;

    // Reset initial value
    // G.resize(18, 18);
    G = Eigen::MatrixXd::Identity(18, 18);

    theta = 0;

    AckermanState.Pos = pos[0];
    AckermanState.Vel = vel[0];
    AckermanState.Qua = qua;
    AckermanPropagation.Pos = pos[0];
    AckermanPropagation.Vel = vel[0];
    AckermanPropagation.Qua = qua;

    encoder_pri = 0.0;

    // debug
    d = 0;

}

void ESKF::LidarMeasurementQuaNoiseScale() {
    std::vector<Eigen::Quaterniond> q_GT;
    int k = 0;
    for (const auto& vec : Attitude_GT) {
        roll_GT.push_back(vec[0]); // 提取每个向量的第一个元素
        pitch_GT.push_back(vec[1]);
        yaw_GT.push_back(vec[2]);
        Eigen::Quaterniond q1 = Eigen::AngleAxisd(yaw_GT[k], Eigen::Vector3d::UnitZ())
                              * Eigen::AngleAxisd(pitch_GT[k], Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd(roll_GT[k], Eigen::Vector3d::UnitX());
        q_GT.push_back(q1);
        k++;
    }
    
    std::vector<Eigen::Quaterniond> q_mea;
    int j = 0;
    for (const auto& vec : Attitude_mea) {
        roll_mea.push_back(vec[0]); // 提取每个向量的第一个元素
        pitch_mea.push_back(vec[1]);
        yaw_mea.push_back(vec[2]);
        Eigen::Quaterniond q2 = Eigen::AngleAxisd(yaw_mea[j], Eigen::Vector3d::UnitZ())
                              * Eigen::AngleAxisd(pitch_mea[j], Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd(roll_mea[j], Eigen::Vector3d::UnitX());
        q_mea.push_back(q2);
        j++;
    }
    int i = 0;
    std::vector<double> w;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    for (auto& q : q_GT) {
        w.push_back(q.w() - q_mea[i].w());
        x.push_back(q.x() - q_mea[i].x());
        y.push_back(q.y() - q_mea[i].y());
        z.push_back(q.z() - q_mea[i].z());
        i++;
    }
    double std_w = MatrixUtils::calculateStandardDeviation(w);
    double std_x = MatrixUtils::calculateStandardDeviation(x);
    double std_y = MatrixUtils::calculateStandardDeviation(y);
    double std_z = MatrixUtils::calculateStandardDeviation(z);
    qua_noise.push_back(std_w);
    qua_noise.push_back(std_x);
    qua_noise.push_back(std_y);
    qua_noise.push_back(std_z);
    // qua_noise.push_back(0.0894); // bug 1
    std::cout << qua_noise[0] << "," << qua_noise[1] << "," << qua_noise[2] << "," << qua_noise[3] << std::endl;



    // std::vector<double> vec;
    // std::ifstream file("/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/vector.csv");
    // std::string line;
    // while (std::getline(file, line)) {
    //     vec.push_back(std::stod(line));
    // }
    // double test = MatrixUtils::calculateStandardDeviation(vec);

    // for (size_t index = 0; index < w.size(); ++index) {
    //     std::vector<double> temp = {w[index], x[index], y[index], z[index]};
    //     qua_noise.push_back(MatrixUtils::calculateStd(temp));
    // }
    // qua_noise.push_back(w);
    // qua_noise.push_back(x);
    // qua_noise.push_back(y);
    // qua_noise.push_back(z);
    // std::cout << "The data size is:" << w.size() << std::endl;
    // for (double val : w) {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;
    // for (double val : x) {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;
    // for (double val : y) {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;
    // for (double val : z) {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;
    // std::cout << ".........................." << qua_noise[0][4] << std::endl;
}

void ESKF::NominalStatePropagation() {
    // int i = Iter;
    Eigen::Vector3d p = NominalState.Pos;
    Eigen::Vector3d v = NominalState.Vel;
    Eigen::Quaterniond q = NominalState.Qua;
    Eigen::Vector3d Acc_b = NominalState.AccBias;
    Eigen::Vector3d Gyro_b = NominalState.GyroBias;
    Eigen::Vector3d g = NominalState.Grav;

    double delta_t = sampling_time;


    // Eigen::MatrixXd R = q.toRotationMatrix();   //用的是eigen的func，可換成自己寫的

    //-----------------------------------------------------------------------------------
    double q_w = q.w();
    Eigen::Vector3d q_v(q.x(), q.y(), q.z());

    // 定义单位矩阵I
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // 计算旋转矩阵R
    Eigen::Matrix3d R = (q_w * q_w - q_v.dot(q_v)) * I
                        + 2 * q_v * q_v.transpose()
                        + 2 * q_w * MatrixUtils::vectorToCrossMatrix(q_v); // 注意：Eigen没有直接的vectorToCrossMatrix()成员函数，需要自定义

    // 打印旋转矩阵R
    // std::cout << "Rotation Matrix R:\n" << R << std::endl;
    //-----------------------------------------------------------------------------------
    // Eigen::Matrix3d R;
    // R = Eigen::AngleAxisd(att[i_imu_update][2], Eigen::Vector3d::UnitZ()) *
    //     Eigen::AngleAxisd(att[i_imu_update][1], Eigen::Vector3d::UnitY()) *
    //     Eigen::AngleAxisd(att[i_imu_update][0], Eigen::Vector3d::UnitX());
    // // 打印旋转矩阵R
    // std::cout << "Rotation Matrix R:\n" << R << std::endl;
    //-----------------------------------------------------------------------------------


    // std::cout << "i_imu:" << i_imu_update << std::endl;
    Eigen::Vector3d NominalState_p_nominal = p + v * delta_t + 0.5 * (R * (Acc_mea[i_imu_update] - Acc_b) + g) * (delta_t * delta_t); // 換回mea
    Eigen::Vector3d NominalState_v_nominal = v + (R * (Acc_mea[i_imu_update] - Acc_b) + g) * delta_t; // 換回mea
    // Eigen::Vector3d NominalState_p_nominal = p + v * delta_t + 0.5 * (R * (Acc_GT[i_imu_update]) + g) * (delta_t * delta_t); // 換回mea
    // Eigen::Vector3d NominalState_v_nominal = v + (R * (Acc_GT[i_imu_update]) + g) * delta_t; // 換回mea
    // std::cout << "NominalState_p_nominal = " << NominalState_p_nominal << std::endl;
    // std::cout << "g = " << g << std::endl;
    // std::cout << "Acc_mea[i_imu_update] = " << Acc_mea[i_imu_update] << std::endl;

    Eigen::Vector3d omega = Omega_mea[i_imu_update] - Gyro_b; // 換回mea
    // Eigen::Vector3d omega = Omega_GT[i_imu_update]; // 換回mea
    double omega_norm = omega.norm();
    Eigen::Quaterniond RotateIncrement;
    if (omega_norm == 0) {
        RotateIncrement = Eigen::Quaterniond(1, 0, 0, 0);
    } else {
        // RotateIncrement = Eigen::Quaterniond(Eigen::AngleAxisd(omega_norm * delta_t * 0.5, omega.normalized())); // 待檢查 // 應該不是這裡的問題，這裡是由角速度得到旋轉增量，再轉換成四元數
        RotateIncrement.w() = cos(omega_norm * 0.5 * delta_t); // 四元数的实部
        RotateIncrement.vec() = omega.normalized() * sin(omega_norm * 0.5 * delta_t); // 四元数的虚部
    }

    Eigen::Quaterniond NominalState_q_nominal = q * RotateIncrement;
    NominalState_q_nominal.normalize(); // 確保結果是單位四元數

    // 封裝結果
    // std::vector<Eigen::Vector3d> NominalState; // 根據需要調整大小
    // NominalState << NominalState_p_nominal, NominalState_v_nominal, Eigen::Vector4d(NominalState_q_nominal.w(), NominalState_q_nominal.x(), NominalState_q_nominal.y(), NominalState_q_nominal.z()), Acc_b, Gyro_b, g;
    NominalState.Pos = NominalState_p_nominal;
    NominalState.Vel = NominalState_v_nominal;
    NominalState.Qua = NominalState_q_nominal;
    NominalState.AccBias = Acc_b;
    NominalState.GyroBias = Gyro_b;
    NominalState.Grav = g;


    // // 更新 NominalState 成員變數
    // // NominalState.resize(6, 1); // 確保 NominalState 有正確的大小
    // state.Pos = NominalState_p_nominal;
    // state.Vel = NominalState_v_nominal;
    // state.Qua = Eigen::Quaterniond(NominalState_q_nominal.w(), NominalState_q_nominal.x(), NominalState_q_nominal.y(), NominalState_q_nominal.z());
    // state.AccBias = Acc_b;
    // state.GyroBias = Gyro_b;
    // state.Grav = g;

    // // return NominalState;

    // // return Eigen::MatrixXd(); // 返回適當的結果
}

// 其他成員函數的實現
void ESKF::ErrorStateTransitionMatrix() {
    // int i = Iter;
    double delta_t = sampling_time;

    Eigen::Vector3d p = State.Pos;
    Eigen::Vector3d v = State.Vel;
    Eigen::Quaterniond q = State.Qua;
    Eigen::Vector3d Acc_b = State.AccBias;
    Eigen::Vector3d Gyro_b = State.GyroBias;
    Eigen::Vector3d g = State.Grav;

    // 定义单位矩阵I
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    // Eigen::MatrixXd R = q.toRotationMatrix();
    //-----------------------------------------------------------------------------------
    double q_w = q.w();
    Eigen::Vector3d q_v(q.x(), q.y(), q.z());

    // 计算旋转矩阵R
    Eigen::Matrix3d R = (q_w * q_w - q_v.dot(q_v)) * I
                        + 2 * q_v * q_v.transpose()
                        + 2 * q_w * MatrixUtils::vectorToCrossMatrix(q_v); // 注意：Eigen没有直接的vectorToCrossMatrix()成员函数，需要自定义

    // 打印旋转矩阵R
    // std::cout << "Rotation Matrix R:\n" << R << std::endl;
    //-----------------------------------------------------------------------------------

    double scale_acc_noise = IMU_noise_scale(0);
    double scale_gyro_noise = IMU_noise_scale(1);
    double scale_acc_bias_noise = IMU_noise_scale(2);
    double scale_gyro_bias_noise = IMU_noise_scale(3);

    Eigen::MatrixXd sigma_vel = I * scale_acc_noise;
    Eigen::MatrixXd sigma_theta = I * scale_gyro_noise;
    Eigen::MatrixXd sigma_acc = I * scale_acc_bias_noise;
    Eigen::MatrixXd sigma_omega = I * scale_gyro_bias_noise;

    // 更新 Fx 和 Qi
    // 注意：在 C++ 中，矩陣索引是從 0 開始的
    Fx.block<3, 3>(3, 9) = -R * delta_t;
    Fx.block<3, 3>(3, 6) = -R * MatrixUtils::vectorToCrossMatrix(Acc_mea[i_imu_update] - Acc_b) * delta_t; // 換回mea
    Sophus::SO3d Fx_expm = Sophus::SO3d::exp(-(Omega_mea[i_imu_update] - Gyro_b) * delta_t); // 換回mea
    // Fx.block<3, 3>(3, 6) = -R * MatrixUtils::vectorToCrossMatrix(Acc_GT[i_imu_update]) * delta_t; // 換回mea
    // Sophus::SO3d Fx_expm = Sophus::SO3d::exp(-(Omega_GT[i_imu_update]) * delta_t); // 換回mea
    Fx.block<3, 3>(6, 6) = Fx_expm.matrix();

    Qi.block<3, 3>(0, 0) = sigma_vel * sigma_vel * delta_t * delta_t * I;
    Qi.block<3, 3>(3, 3) = sigma_theta * sigma_theta * delta_t * delta_t * I;
    Qi.block<3, 3>(6, 6) = sigma_acc * sigma_acc * delta_t * I;
    Qi.block<3, 3>(9, 9) = sigma_omega * sigma_omega * delta_t * I;
    
    // 不需要返回值，因為 Fx 和 Qi 是通過引用傳遞的
}

// void ESKF::GPSMeasurementTransitionMatrix(Eigen::MatrixXd& Hx, Eigen::MatrixXd& V, Eigen::MatrixXd& Xdeltax, const NominalState& Nominal_state, const Eigen::VectorXd& noise_scale, const Eigen::VectorXd& qua_noise) {
//     Eigen::Quaterniond q_nominal = Nominal_state.q_nominal;
    
//     Eigen::MatrixXd Qdeltatheta(4, 3);
//     Qdeltatheta << -q_nominal.x(), -q_nominal.y(), -q_nominal.z(),
//                     q_nominal.w(), -q_nominal.z(), q_nominal.y(),
//                     q_nominal.z(), q_nominal.w(), -q_nominal.x(),
//                    -q_nominal.y(), q_nominal.x(), q_nominal.w();
//     Xdeltax.block<4, 3>(6, 6) = Qdeltatheta;

//     double scale_translation_noise = noise_scale(0);
//     double scale_rotation_noise = noise_scale(1);

//     Eigen::MatrixXd sigma_translation = Eigen::MatrixXd::Identity(3, 3) * scale_translation_noise;
//     // Eigen::MatrixXd sigma_rotation = Eigen::MatrixXd::Identity(4, 4) * scale_rotation_noise;
//     // 根據需求啟用或修改上面的 sigma_rotation 計算

//     V.block<3, 3>(0, 0) = sigma_translation.cwiseProduct(sigma_translation);
//     // V.block<4, 4>(3, 3) = sigma_rotation.cwiseProduct(sigma_rotation);

//     Hx = Hx * Xdeltax;
//     // 不需要返回值，因為 Hx 和 V 是通過引用傳遞的
// }

void ESKF::LidarMeasurementTransitionMatrix() {
    Eigen::Quaterniond q = NominalState.Qua;

    Eigen::MatrixXd Qdeltatheta(4, 3);
    Qdeltatheta << -q.x(), -q.y(), -q.z(),
                    q.w(), -q.z(), q.y(),
                    q.z(), q.w(), -q.x(),
                   -q.y(), q.x(), q.w();
    Xdeltax.block<4, 3>(6, 6) = 0.5 * Qdeltatheta;

    double scale_translation_noise = Lidar_noise_scale(0);
    double scale_rotation_noise = Lidar_noise_scale(1);

    Eigen::MatrixXd sigma_translation = Eigen::MatrixXd::Identity(3, 3) * scale_translation_noise;
    Eigen::MatrixXd sigma_rotation = Eigen::MatrixXd::Identity(4, 4);
    for (int j = 0; j < 4; ++j) {
        // std::cout << scale_rotation_noise << std::endl;
        // std::cout << "i_slam_update = " << i_slam_update << std::endl;
        // std::cout << "j = " << j << std::endl;
        // std::cout << qua_noise[j] << std::endl;
        sigma_rotation(j, j) = scale_rotation_noise * qua_noise[j];
    }
    // std::cout << sigma_rotation << std::endl;

    V1.block<3, 3>(0, 0) = sigma_translation.cwiseProduct(sigma_translation);
    V1.block<4, 4>(3, 3) = sigma_rotation.cwiseProduct(sigma_rotation);
    // std::cout << Xdeltax << std::endl;

    H1 = Hx1 * Xdeltax;
    // std::cout << H1 << std::endl;
    // 不需要返回值，因為 Hx 和 V 是通過引用傳遞的
}

void ESKF::AckermanMeasurementTransitionMatrix() {
    Eigen::Quaterniond q = NominalState.Qua;

    Eigen::MatrixXd Qdeltatheta(4, 3);
    Qdeltatheta << -q.x(), -q.y(), -q.z(),
                    q.w(), -q.z(), q.y(),
                    q.z(), q.w(), -q.x(),
                   -q.y(), q.x(), q.w();
    Xdeltax.block<4, 3>(6, 6) = 0.5 * Qdeltatheta;

    // double scale_vel_noise = Ackerman_noise_scale(0);
    // double scale_steering_noise = Ackerman_noise_scale(1);

    // Eigen::MatrixXd sigma_vel = Eigen::MatrixXd::Identity(4, 4) * scale_vel_noise * 0.01; // 0.01 為積分的 delta_t
    // Eigen::MatrixXd sigma_attitude = Eigen::MatrixXd::Identity(4, 4) * scale_steering_noise;

    // V.block<4, 4>(0, 0) = sigma_vel.cwiseProduct(sigma_vel);
    // V.block<4, 4>(4, 4) = sigma_attitude.cwiseProduct(sigma_attitude);

    H2 = Hx2 * Xdeltax;
    xtrue2err = Xdeltax;
    // std::cout << "The matrix size is: " << xtrue2err.rows() << "x" << xtrue2err.cols() << std::endl;
    // std::cout << "H2 is: " << H2 << std::endl;
    // 不需要返回值，因為 Hx 和 V 是通過引用傳遞的
}

void ESKF::Injection() {
    NominalState.Pos += XFusion.block<3, 1>(0, 0);
    NominalState.Vel += XFusion.block<3, 1>(3, 0);

    Eigen::Vector3d delta_theta = XFusion.block<3, 1>(6, 0);
    if (delta_theta.norm() == 0) { // norm = 0的話 下面式子異常解 所以有這個判斷式
        // 不需要改變 q_nominal
    } else {
        Eigen::Quaterniond delta_q(Eigen::AngleAxisd(delta_theta.norm(), delta_theta.normalized()));
        NominalState.Qua = NominalState.Qua * delta_q;
    }

    NominalState.AccBias += XFusion.block<3, 1>(9, 0);
    NominalState.GyroBias += XFusion.block<3, 1>(12, 0);
    NominalState.Grav += XFusion.block<3, 1>(15, 0);
}

void ESKF::ResetTransitionMatrix() {
    // Eigen::Matrix3d G_7x9 = Eigen::Matrix3d::Identity() - MatrixUtils::vectorToCrossMatrix(0.5 * XFusion.block<3, 1>(6, 0));
    // 或者，如果您需要使用 expm 函數，您可能需要自己實現它，因為 Eigen 沒有直接提供
    // Eigen::Matrix3d G_7x9 = expm(-MatrixUtils::vectorToCrossMatrix(0.5 * Error_state.XFusion.block<3, 1>(6, 0)));
    Sophus::SO3d G_expm = Sophus::SO3d::exp(0.5 * XFusion.block<3, 1>(6, 0));
    Eigen::Matrix3d G_7x9 = G_expm.matrix();

    G.block<3, 3>(6, 6) = G_7x9;

    P = G * P * G.transpose();
}

void ESKF::LidarFusionProcess(){
    // 要加入判別是否有Lidar資料(if)
    LidarMeasurementTransitionMatrix();

    

    // Lidar State Update
    Eigen::Vector3d Lidar_pos = Position_mea[i_slam_update]; // 換回mea
    Eigen::Quaterniond Lidar_qua = Eigen::AngleAxisd(yaw_mea[i_slam_update], Eigen::Vector3d::UnitZ())
                                 * Eigen::AngleAxisd(pitch_mea[i_slam_update], Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(roll_mea[i_slam_update], Eigen::Vector3d::UnitX()); // 換回mea
    // std::cout << "Lidar_qua = " << Lidar_qua.w() << "," << Lidar_qua.x() << "," << Lidar_qua.y() << "," << Lidar_qua.z() << std::endl;
    // Eigen::Vector3d Lidar_pos = Position_GT[i_slam_update]; // 換回mea
    // Eigen::Quaterniond Lidar_qua = Eigen::AngleAxisd(yaw_GT[i_slam_update], Eigen::Vector3d::UnitZ())
    //                              * Eigen::AngleAxisd(pitch_GT[i_slam_update], Eigen::Vector3d::UnitY())
    //                              * Eigen::AngleAxisd(roll_GT[i_slam_update], Eigen::Vector3d::UnitX()); // 換回mea
    Eigen::Quaterniond measured_qua = Lidar_qua;
    Eigen::Quaterniond predicted_qua = NominalState.Qua;
    double dot_product = measured_qua.coeffs().dot(predicted_qua.coeffs());
    if (dot_product < 0) {
        Lidar_qua.coeffs() = -Lidar_qua.coeffs(); // 取反所有系数
        d++;
    }
    Eigen::Quaterniond q_error(Lidar_qua.coeffs() - NominalState.Qua.coeffs());
    Eigen::Vector3d p_error = Lidar_pos - NominalState.Pos;

    K1.setZero();
    K1 = P*H1.transpose()*(H1*P*H1.transpose() + V1).inverse();
    Eigen::VectorXd err_vector(7);
    err_vector << p_error, q_error.w(), q_error.x(), q_error.y(), q_error.z();
    // std::cout << "err_vector = " << err_vector << std::endl;
    XFusion = K1 * err_vector;
    // std::cout << "H1 = " << H1 << std::endl;
    // std::cout << "P = " << P << std::endl;
    // std::cout << "V1 = " << V1 << std::endl;
    // std::cout << "P*H1.transpose() = " << P*H1.transpose() << std::endl;
    // std::cout << "(H1*P*H1.transpose() + V1).inverse() = " << (H1*P*H1.transpose() + V1).inverse() << std::endl;
    // std::cout << "H1.transpose() = " << H1.transpose() << std::endl;
    // std::cout << "K1 = " << K1 << std::endl;
    // std::cout << "dot = " << measured_qua.coeffs().dot(predicted_qua.coeffs()) << std::endl;
    // std::cout << "dot = " << measured_qua.dot(predicted_qua) << std::endl;
    // if (measured_qua.dot(predicted_qua) != dot_product) {
    //     std::cerr << "條件不滿足，程序將終止。" << std::endl;
    //     std::exit(EXIT_FAILURE); // 使用非零值表示異常終止
    // }
    // std::cout << "err_vector = " << err_vector << std::endl;
    // std::cout << "XFusion = " << XFusion << std::endl;
    Eigen::MatrixXd KH = K1 * H1;
    Eigen::MatrixXd identityMatrix = Eigen::MatrixXd::Identity(KH.rows(), KH.cols());
    P = (identityMatrix - KH) * P;

    Injection();

    ResetTransitionMatrix();

    // 為了後面阿克曼協方差更新用的
    P_fusion = P;

}

void ESKF::AckermanFusionProcess(){
    AckermanMeasurementTransitionMatrix();

    V2(0, 0) = 0.5;
    V2(1, 1) = 0.5;

    // Ackerman State Update
    AckermanBaseMea.vel = (vel_count_mea[i_ack_update] / rear_wheel_count) * 2.0 * M_PI / sampling_time_ack;
    AckermanBaseMea.steer = (steer_count_mea[i_ack_update] / heading_angle_count) * 2.0 * M_PI;
    // AckermanBaseMea.vel = (vel_count_GT[i_ack_update] / rear_wheel_count) * 2.0 * M_PI / sampling_time_ack; // 換回mea
    // AckermanBaseMea.steer = (steer_count_GT[i_ack_update] / heading_angle_count) * 2.0 * M_PI; // 換回mea

    double* arrayP = MatrixUtils::EigenMatrixToArray(P_fusion);
    double* arrayV = MatrixUtils::EigenMatrixToArray(V2);
    double* arrayxtrue2err = MatrixUtils::EigenMatrixToArray(xtrue2err);
    double R_mea[36];
    const struct0_T *ptrToAckermanBaseMea = &AckermanBaseMea; // 創建一個指向myStruct的const指針


    MeaCov2C(arrayP, arrayV,
            arrayxtrue2err, ptrToAckermanBaseMea,
            encoder_pri, theta,
            Ackermanparam, sampling_time_ack,
            R_mea);
    // 在這次更新完之後 theta 會更新

    // for(int i = 0; i < 36; i++) {
    //     std::cout << R_mea[i] << std::endl; // 或者使用其他格式化方式
    // }
    // 使用Eigen::Map将R转换为6x6的Eigen::MatrixXd
    Eigen::Map<Eigen::Matrix<double, 6, 6>> R_matrix(R_mea);
    // std::cout << "R_matrix:\n" << R_matrix << std::endl;

    // 获取对角线元素
    Eigen::VectorXd diag = R_matrix.diagonal();

    // 将对角线元素转换为对角矩阵
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> V_diag(diag);
    Eigen::MatrixXd denseV = V_diag.toDenseMatrix();
    // std::cout << "V_ack = " << denseV << std::endl;

    AckermanStatePropagation();

    Eigen::Quaterniond measured_qua = AckermanState.Qua;
    Eigen::Quaterniond predicted_qua = NominalState.Qua;
    double dot_product = measured_qua.dot(predicted_qua);
    if (dot_product < 0) {
        AckermanState.Qua.coeffs() = -AckermanState.Qua.coeffs(); // 取反所有系数
    }
    Eigen::Quaterniond q_error(AckermanState.Qua.coeffs() - NominalState.Qua.coeffs());
    Eigen::Vector3d v_error = AckermanState.Vel - NominalState.Vel;

    K2.setZero();
    K2 = P*H2.transpose()*(H2*P*H2.transpose() + denseV).inverse();
    Eigen::VectorXd err_vector(6);
    err_vector << v_error[0], v_error[1], q_error.w(), q_error.x(), q_error.y(), q_error.z();
    XFusion = K2 * err_vector;
    Eigen::MatrixXd KH = K2 * H2;
    Eigen::MatrixXd identityMatrix = Eigen::MatrixXd::Identity(KH.rows(), KH.cols());
    P = (identityMatrix - KH) * P;

    Injection();

    ResetTransitionMatrix();

    P_fusion = P;

    // 此處這步驟跟lidar不一樣的地方在於，阿克曼的狀態會基於前一次的數據，加上量測到的增量求得，而lidar的狀態直接基於當下lidar的量測值求得，所以這邊需要多一個步驟紀錄阿克曼前一次的狀態
    AckermanState.Vel = NominalState.Vel;
    AckermanState.Qua = NominalState.Qua;
    Eigen::Vector3d Ack_theta = MatrixUtils::quaternionToEulerAngles(AckermanState.Qua);
    theta = Ack_theta[2];
    // std::cout << "The H2 size is: " << H2.rows() << "x" << H2.cols() << std::endl;
    // std::cout << "The P size is: " << P.rows() << "x" << P.cols() << std::endl;
    // std::cout << "The denseV size is: " << denseV.rows() << "x" << denseV.cols() << std::endl;
    // std::cout << "H2 is: " << H2 << std::endl;
    // std::cout << "P is: " << P << std::endl;
    // std::cout << "denseV is: " << denseV << std::endl;
}

// 简化的状态传播函数
void ESKF::AckermanStatePropagation() {
    double L = Ackermanparam[0]*0.001; // 示例值，应根据实际情况进行调整
    double L1 = Ackermanparam[1]*0.001; // 示例值
    double L2 = Ackermanparam[2]*0.001;
    double L3 = Ackermanparam[3]*0.001;
    double L4 = Ackermanparam[4]*0.001;
    double l_rear = Ackermanparam[5]*0.001;
    double l_ax = Ackermanparam[6]*0.001;
    double r_wheel = Ackermanparam[7]*0.001;
    double a = atan2(L, L1 / 2.0);
    double delta_r = encoder_pri + AckermanBaseMea.steer;
    double omega_k = AckermanBaseMea.vel;
    // std::cout << "L = " << L << std::endl;
    // std::cout << "L1 = " << L1 << std::endl;
    // std::cout << "L2 = " << L2 << std::endl;
    // std::cout << "L3 = " << L3 << std::endl;
    // std::cout << "L4 = " << L4 << std::endl;
    // std::cout << "l_rear = " << l_rear << std::endl;
    // std::cout << "l_ax = " << l_ax << std::endl;
    // std::cout << "r_wheel = " << r_wheel << std::endl;
    // std::cout << "a = " << a << std::endl;
    // std::cout << "delta_r = " << delta_r << std::endl;
    // std::cout << "omega_k = " << omega_k << std::endl;
    // std::cout << "encoder_pri = " << encoder_pri << std::endl;
    // std::cout << "AckermanBaseMea.steer = " << AckermanBaseMea.steer << std::endl;

    // 四元数转欧拉角
    // double roll, pitch, yaw;
    Eigen::Vector3d vec_ack_ang = MatrixUtils::quaternionToEulerAngles(AckermanState.Qua); // 沒問題

    // std::cout << "vec_ack_ang = " << vec_ack_ang <<std::endl;

    double heading_angle = vec_ack_ang[2];
    // std::cout << "heading_angle = " << heading_angle << std::endl;

    double x = AckermanState.Pos[0];
    double y = AckermanState.Pos[1];
    double v_x = AckermanState.Vel[0];
    double v_y = AckermanState.Vel[1];
    // std::cout << "x = " << x << "y = " << y << std::endl;
    // std::cout << "vx = " << v_x << "vy = " << v_y << std::endl;

    double a_r = delta_r + a;
    // std::cout << "a_r = " << a_r << std::endl;

    double S = sqrt(L1 * L1 + L4 * L4 - 2 * L1 * L4 * cos(a_r));
    double b = acos((L1 * L1 + S * S - L4 * L4) / (2.0 * L1 * S));
    double c = acos((L2 * L2 + S * S - L3 * L3) / (2.0 * L2 * S));
    double a_l = b + c;
    // std::cout << "S = " << S << std::endl;
    // std::cout << "b = " << b << std::endl;
    // std::cout << "c = " << c << std::endl;
    // std::cout << "a_l = " << a_l << std::endl;

    double delta_l = a - a_l;
    double delta_f = MatrixUtils::acot(MatrixUtils::cot(delta_r) -(((l_rear / 2.0) - l_ax) / ((l_rear - 2.0 * l_ax) / (MatrixUtils::cot(delta_r) - MatrixUtils::cot(delta_l)))));
    double R_m = MatrixUtils::sign(delta_r) * L * MatrixUtils::cot(delta_f);
    // std::cout << "delta_l = " << delta_l << std::endl;
    // std::cout << "delta_f = " << delta_f << std::endl;
    // std::cout << "R_m = " << R_m << std::endl;
    // std::cout << "MatrixUtils::sign(delta_r) = " << MatrixUtils::sign(delta_r) << std::endl;
    // std::cout << "MatrixUtils::cot(delta_f) = " << MatrixUtils::cot(0.0146477) << std::endl;

    double omega_l, omega_r, omega_B, V_r, V_f;
    if (delta_r == 0.0) {
        omega_l = omega_k;
        omega_r = omega_k;
        V_r = omega_k * r_wheel;
        V_f = V_r;
        omega_B = 0.0;
        delta_f = 0.0;
    } else {
        omega_l = (2.0 * omega_k * ((R_m - MatrixUtils::sign(delta_r) * (l_rear / 2.0)) / (R_m + MatrixUtils::sign(delta_r) * (l_rear / 2.0)))) / (1.0 + (((R_m - MatrixUtils::sign(delta_r) * (l_rear / 2.0)) / (R_m + MatrixUtils::sign(delta_r) * (l_rear / 2.0)))));
        omega_r = 2.0 * omega_k - omega_l;
        omega_B = omega_l * r_wheel / ((R_m - MatrixUtils::sign(delta_r) * (l_rear / 2.0)) * MatrixUtils::sign(delta_r));
        V_r = R_m * MatrixUtils::sign(delta_r) * omega_B;
        V_f = R_m * (1.0 / cos(delta_f)) * MatrixUtils::sign(delta_r) * omega_B; // sec(delta_f) is 1/cos(delta_f)
    }
    // std::cout << "omega_l = " << omega_l << std::endl;
    // std::cout << "omega_r = " << omega_r << std::endl;
    // std::cout << "omega_B = " << omega_B << std::endl;
    // std::cout << "V_r = " << V_r << std::endl;
    // std::cout << "V_f = " << V_f << std::endl;
    // std::cout << "delta_f = " << delta_f << std::endl;
    
    x += V_r * cos(heading_angle) * sampling_time_ack;
    y += V_r * sin(heading_angle) * sampling_time_ack;
    v_x = V_r * cos(heading_angle);
    v_y = V_r * sin(heading_angle);
    heading_angle += omega_B * sampling_time_ack;
    // std::cout << "x = " << x << std::endl;
    // std::cout << "y = " << y << std::endl;
    // std::cout << "v_x = " << v_x << std::endl;
    // std::cout << "v_y = " << v_y << std::endl;
    // std::cout << "heading_angle = " << heading_angle << std::endl;

    AckermanState.Pos = Eigen::Vector3d(x, y, 0.0);
    AckermanState.Vel = Eigen::Vector3d(v_x, v_y, 0.0);
    Eigen::AngleAxisd angleAxis(heading_angle, Eigen::Vector3d::UnitZ());
    AckermanState.Qua = Eigen::Quaterniond(angleAxis);
    // std::cout << "heading_angle = " << V_r * cos(heading_angle) * sampling_time_ack << std::endl;

    encoder_pri = delta_r;
    // std::cout << "encoder_pri = " << encoder_pri << std::endl;

    // debug
    ack_vel_x.push_back(v_x);
    ack_vel_y.push_back(v_y);
    ack_vel_z.push_back(0);
    


}

void ESKF::loadFromJson(const std::vector<std::string>& filename) {

    // 假設 JSON 結構如下：
    // {
    //     "Omega_m": [[...], [...], ...],
    //     "Acc_m": [[...], [...], ...],
    //     "GPSData": [[...], [...], ...]
    // }
    
    //-------------------------------------------------------------------------------
    // 使用 ifstream 打开 JSON 文件
    std::ifstream file(filename[0]);
    if (!file.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
    }

    // 使用 nlohmann::json 对象解析文件
    nlohmann::json IMUData;
    file >> IMUData;

    // 关闭文件
    file.close();

    // 假设我们使用 std::vector 来存储数据
    std::vector<std::vector<double>> Acc_GT_data = IMUData["IMUData"]["Acc_GT"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> Omega_GT_data = IMUData["IMUData"]["Omega_GT"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> Acc_mea_data = IMUData["IMUData"]["Acc_mea"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> Omega_mea_data = IMUData["IMUData"]["Omega_mea"].get<std::vector<std::vector<double>>>();
    //-------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------
    // 使用 ifstream 打开 JSON 文件
    std::ifstream file1(filename[1]);
    if (!file1.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
    }

    // 使用 nlohmann::json 对象解析文件
    nlohmann::json LidarData;
    file1 >> LidarData;

    // 关闭文件
    file1.close();

    // 假设我们使用 std::vector 来存储数据
    std::vector<std::vector<double>> Position_GT_data = LidarData["LidarData"]["Position_GT"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> Attitude_GT_data = LidarData["LidarData"]["Attitude_GT"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> Position_mea_data = LidarData["LidarData"]["Position_mea"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> Attitude_mea_data = LidarData["LidarData"]["Attitude_mea"].get<std::vector<std::vector<double>>>();
    //-------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------
    // 使用 ifstream 打开 JSON 文件
    std::ifstream file2(filename[2]);
    if (!file2.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
    }

    // 使用 nlohmann::json 对象解析文件
    nlohmann::json EncoderData;
    file2 >> EncoderData;

    // 关闭文件
    file2.close();

    // 假设我们使用 std::vector 来存储数据
    vel_count_GT = EncoderData["EncoderData"]["vel_count_GT"].get<std::vector<double>>();
    steer_count_GT = EncoderData["EncoderData"]["steer_count_GT"].get<std::vector<double>>();
    vel_count_mea = EncoderData["EncoderData"]["vel_count_mea"].get<std::vector<double>>();
    steer_count_mea = EncoderData["EncoderData"]["steer_count_mea"].get<std::vector<double>>();
    // for (double val : steer_count_GT) {
    //     std::cout << val << " ";
    // }
    // if (!vel_count_mea.empty()) { // 检查vector是否为空
    //     double firstElement = vel_count_mea[0]; // 获取第一个元素
    //     // 现在你可以使用firstElement变量了，它是一个double类型的值
    //     std::cout << "第一个元素是: " << firstElement << std::endl;
    // } else {
    //     // 如果vector为空，输出错误消息
    //     std::cout << "错误：vector为空，无法获取第一个元素。" << std::endl;
    // }
    //-------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------
    // 使用 ifstream 打开 JSON 文件
    std::ifstream file3(filename[3]);
    if (!file3.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
    }

    // 使用 nlohmann::json 对象解析文件
    nlohmann::json GroundTruthData;
    file3 >> GroundTruthData;

    // 关闭文件
    file3.close();

    // 假设我们使用 std::vector 来存储数据
    std::vector<std::vector<double>> pos_data = GroundTruthData["GTData"]["pos"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> vel_data = GroundTruthData["GTData"]["vel"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> att_data = GroundTruthData["GTData"]["att"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> accb_data = GroundTruthData["GTData"]["accb"].get<std::vector<std::vector<double>>>();
    std::vector<std::vector<double>> gyrob_data = GroundTruthData["GTData"]["gyrob"].get<std::vector<std::vector<double>>>();
    //-------------------------------------------------------------------------------

    // std::cout << Acc_GT_data[0][0] << std::endl;

    // if (!Acc_GT_data.empty()) {
    //     std::cout << "Acc_GT 第一组向量: ";
    //     for (double value : Acc_GT_data[0]) {
    //         std::cout << value << " ";
    //     }
    //     std::cout << std::endl;
    // } else {
    //     std::cout << "Acc_GT 是空的或未正确加载数据。" << std::endl;
    // }

    // Eigen::MatrixXd Acc_GT_Matrix = ConvertToEigenMatrix(Acc_GT);
    Acc_GT = ConvertToEigenVector3d(Acc_GT_data);
    Omega_GT = ConvertToEigenVector3d(Omega_GT_data);
    Acc_mea = ConvertToEigenVector3d(Acc_mea_data);
    Omega_mea = ConvertToEigenVector3d(Omega_mea_data);
    Position_GT = ConvertToEigenVector3d(Position_GT_data);
    Attitude_GT = ConvertToEigenVector3d(Attitude_GT_data);
    Position_mea = ConvertToEigenVector3d(Position_mea_data);
    Attitude_mea = ConvertToEigenVector3d(Attitude_mea_data);
    pos = ConvertToEigenVector3d(pos_data);
    vel = ConvertToEigenVector3d(vel_data);
    att = ConvertToEigenVector3d(att_data);
    accb = ConvertToEigenVector3d(accb_data);
    gyrob = ConvertToEigenVector3d(gyrob_data);

    std::cout << Omega_GT[1000] << std::endl;
}

Eigen::MatrixXd ESKF::ConvertToEigenMatrix(const std::vector<std::vector<double>>& data) {
    if (data.empty()) return Eigen::MatrixXd();

    size_t rows = data.size();
    size_t cols = data.front().size();
    Eigen::MatrixXd mat(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            mat(i, j) = data[i][j];
        }
    }
    return mat;
}

std::vector<Eigen::Vector3d> ESKF::ConvertToEigenVector3d(const std::vector<std::vector<double>>& input) {
    std::vector<Eigen::Vector3d> result;
    for (const auto& vec : input) {
        if (vec.size() >= 3) {
            // 只取前三个元素
            Eigen::Vector3d eigenVec(vec[0], vec[1], vec[2]);
            result.push_back(eigenVec);
        }
        // 如果 vec 的大小小于 3，你可以选择忽略该 vec 或者用默认值填充
    }
    return result;
}

Eigen::Quaterniond ESKF::eulerToQuaternion(double roll, double pitch, double yaw) {
    // std::cout << roll << std::endl;
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

void ESKF::runESKF() {
    ValueInitialize();
    LidarMeasurementQuaNoiseScale();
    // 處理循環(ESKF)
    // (size_t i = 0; i < end_time / sampling_time - 1; i++)
    for (size_t i = 0; i < end_time / sampling_time - 1; i++){
        // (end_time / sampling_time - 1)/10
        // std::cout << "iter times:" << i_imu_update <<std::endl;
        NominalStatePropagation();
        ErrorStateTransitionMatrix();

        // State Prediction
        XErrorPre = Fx*XError; // std::cout << "XError = " << XError << std::endl;
        P = Fx*P*Fx.transpose() + Fi*Qi*Fi.transpose();
        // std::cout << Fx << std::endl;


        if (std::round(t[i_imu_update+1] * 1000) / 1000 == std::round(t_slam[i_slam_update] * 1000) / 1000) {
            // 如果t[i]和t_ack[j]在小数点后三位四舍五入后相等，则执行这里的代码
            LidarFusionProcess();
            // std::cout << "lidar iter times:" << i_slam_update << std::endl;
            i_slam_update++;
        }else{
            XFusion = Eigen::VectorXd::Zero(18);
        }
        
        if (std::round(t[i_imu_update] * 1000) / 1000 == std::round(t_ack[i_ack_update] * 1000) / 1000) {
            // 如果t[i]和t_ack[j]在小数点后三位四舍五入后相等，则执行这里的代码
            AckermanFusionProcess();
            // std::cout << "Ack iter times:" << i_ack_update << std::endl;
            //------------------------------------------------------------------------
            // AckermanBaseMea.vel = (vel_count_mea[i_ack_update] / rear_wheel_count) * 2 * M_PI / sampling_time_ack;
            // AckermanBaseMea.steer = (steer_count_mea[i_ack_update] / heading_angle_count) * 2 * M_PI;
            // AckermanStatePropagation();
            //------------------------------------------------------------------------
            i_ack_update++;
        }else{
            XFusion = Eigen::VectorXd::Zero(18);
        }

        Eigen::Vector3d CurAtt = MatrixUtils::quaternionToEulerAngles(NominalState.Qua);

        Fusion_Pose_Data.push_back(NominalState.Pos);
        Fusion_Vel_Data.push_back(NominalState.Vel);
        Fusion_Att_Data.push_back(CurAtt);
        Fusion_AccBias_Data.push_back(NominalState.AccBias);
        Fusion_GyroBias_Data.push_back(NominalState.GyroBias);
        i_imu_update = i+1;
        // std::cout << "P Cov:" << P <<std::endl;
        // std::cout << "Pos:" << AckermanPropagation.Pos << std::endl << "iter = " << i_imu_update << std::endl;
        State = NominalState;
    }

    // Ackerman propagation
    AckermanState = AckermanPropagation;
    encoder_pri = 0.0;
    std::vector<double> x_starts;
    std::vector<double> y_starts;
    std::vector<double> z_starts;
    for (size_t i = 0; i < end_time / sampling_time - 1; i++){
        AckermanBaseMea.vel = (vel_count_mea[i] / rear_wheel_count) * 2 * M_PI / sampling_time_ack;
        AckermanBaseMea.steer = (steer_count_mea[i] / heading_angle_count) * 2 * M_PI;
        // std::cout << "AckermanBaseMea.vel = " << AckermanBaseMea.vel << std::endl;
        // std::cout << "AckermanBaseMea.steer = " << AckermanBaseMea.steer << std::endl;
        
        x_starts.push_back(AckermanState.Pos[0]);
        y_starts.push_back(AckermanState.Pos[1]);
        z_starts.push_back(AckermanState.Pos[2]);
        AckermanStatePropagation();

        // Fusion_Pose_Data.push_back(AckermanState.Pos);
        // std::cout << "Pos:" << AckermanState.Pos << std::endl << "iter = " << i << std::endl;

    }
    // x_starts.pop_back();y_starts.pop_back();
    // plt::quiver(x_starts, y_starts, ack_vel_x, ack_vel_y);
    
    // SLAM propagation
    for (size_t i = 0; i < end_time / sampling_time_slam - 1; i++){
        // Lidar State Update
        AckermanState.Pos = Position_mea[i]; // 換回mea
        AckermanState.Qua = Eigen::AngleAxisd(yaw_mea[i], Eigen::Vector3d::UnitZ())
                                 * Eigen::AngleAxisd(pitch_mea[i], Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(roll_mea[i], Eigen::Vector3d::UnitX());
        
        // Fusion_Pose_Data.push_back(AckermanState.Pos);
    }




    // std::cout << "d = " << d << std::endl;
    
    // std::cout << "iter times:" << i_slam_update <<std::endl;
    // std::cout << "iter times:" << i_ack_update <<std::endl;
    // for (int val : t_slam) {
    //     std::cout << val << " ";
    // }

    // IMU propagation
    NominalState = NominalPropagation;
    i_imu_update = 0;
    for (size_t i = 0; i < end_time / sampling_time - 1; ++i) {
        NominalStatePropagation();
        // Fusion_Pose_Data.push_back(NominalState.Pos);
        i_imu_update++;
    }



    std::vector<double> fusion_pos_x;
    std::vector<double> fusion_pos_y;
    std::ofstream file1("Fusion_Pose_Data.txt");
    for(const auto& vec : Fusion_Pose_Data) {
        file1 << vec[0] << " " << vec[1] << " " << vec[2] << "\n";
        fusion_pos_x.push_back(vec[0]);
        fusion_pos_y.push_back(vec[1]);
    }
    file1.close();
    

    // matplotlib
    // 初始化兩個std::vector<double>來存儲所有x和y坐標
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    std::vector<double> z_coords;
    for(const auto& pos : Fusion_Pose_Data) {
        x_coords.push_back(pos(0)); // 添加x坐標
        y_coords.push_back(pos(1)); // 添加y坐標
        z_coords.push_back(pos(2));
    }
    std::vector<double> index_vector(x_coords.size());

    // 初始化index_vector的值
    for(size_t i = 0; i < index_vector.size(); ++i) {
        index_vector[i] = static_cast<double>(i);
    }
    plt::figure();
    plt::grid(true);plt::axis("equal");
    plt::title("Fusion Result");
    plt::plot(x_coords, y_coords);

    // velocity
    std::vector<double> vx_coords;
    std::vector<double> vy_coords;
    std::vector<double> vz_coords;
    std::ofstream file2("Fusion_Vel_Data.txt");
    for(const auto& vel : Fusion_Vel_Data) {
        file2 << vel[0] << " " << vel[1] << " " << vel[2] << "\n";
        vx_coords.push_back(vel(0));
        vy_coords.push_back(vel(1));
        vz_coords.push_back(vel(2));
    }
    file2.close();

    plt::figure();
    plt::subplot(2, 3, 1);
    plt::grid(true);
    plt::title("Vx");
    plt::plot(index_vector, vx_coords);

    plt::subplot(2, 3, 2);
    plt::grid(true);
    plt::title("Vy");
    plt::plot(index_vector, vy_coords);

    plt::subplot(2, 3, 3);
    plt::grid(true);
    plt::title("Vz");
    plt::plot(index_vector, vz_coords);

    // attitude
    std::vector<double> attx_coords;
    std::vector<double> atty_coords;
    std::vector<double> attz_coords;
    std::ofstream file3("Fusion_Att_Data.txt");
    for(const auto& att : Fusion_Att_Data) {
        file3 << att[0] << " " << att[1] << " " << att[2] << "\n";
        attx_coords.push_back(att(0));
        atty_coords.push_back(att(1));
        attz_coords.push_back(att(2));
    }
    file3.close();

    // AccBias
    std::vector<double> AccBiasx_coords;
    std::vector<double> AccBiasy_coords;
    std::vector<double> AccBiasz_coords;
    std::ofstream file4("Fusion_AccBias_Data.txt");
    for(const auto& vec : Fusion_AccBias_Data) {
        file4 << vec[0] << " " << vec[1] << " " << vec[2] << "\n";
        AccBiasx_coords.push_back(vec(0));
        AccBiasy_coords.push_back(vec(1));
        AccBiasz_coords.push_back(vec(2));
    }
    file4.close();

    // GyroBias
    std::vector<double> GyroBiasx_coords;
    std::vector<double> GyroBiasy_coords;
    std::vector<double> GyroBiasz_coords;
    std::ofstream file5("Fusion_GyroBias_Data.txt");
    for(const auto& vec : Fusion_GyroBias_Data) {
        file5 << vec[0] << " " << vec[1] << " " << vec[2] << "\n";
        GyroBiasx_coords.push_back(vec(0));
        GyroBiasy_coords.push_back(vec(1));
        GyroBiasz_coords.push_back(vec(2));
    }
    file5.close();

    plt::subplot(2, 3, 4);
    plt::grid(true);
    plt::title("roll");
    plt::plot(index_vector, attx_coords);

    plt::subplot(2, 3, 5);
    plt::grid(true);
    plt::title("pitch");
    plt::plot(index_vector, atty_coords);

    plt::subplot(2, 3, 6);
    plt::grid(true);
    plt::title("yaw");
    plt::plot(index_vector, attz_coords);
    
    plt::show();
}

int main(){
    Eigen::MatrixXd Omega_m; // 初始化 Omega_m
    Eigen::MatrixXd Acc_m;   // 初始化 Acc_m
    Eigen::MatrixXd GPS;     // 初始化 GPS
    int iter = 0;            // 初始化 iter
    // double sampling_time = 0.01; // 設置採樣時間
    ESKF eskf(Omega_m, Acc_m, GPS, iter);

    // std::string filename = "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/IMUData.json";
    std::vector<std::string> filename = {
        "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/IMUData.json",
        "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/LidarData.json",
        "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/EncoderData.json",
        "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/GroundTruthData.json"
    };


    eskf.loadFromJson(filename);

    eskf.runESKF();

    //-------------------------------------------------------------------------------------------------------------------------
    // github上的expm用法
    // boost::numeric::ublas::bmatrix A(3, 3);
    // A(0,0) = 1; A(0,1) = 2; A(0,2) = 3;
    // A(1,0) = 4; A(1,1) = 5; A(1,2) = 6;
    // A(2,0) = 7; A(2,1) = 8; A(2,2) = 9;

    // // 打印原始矩陣
    // std::cout << "原始矩陣 A:\n" << A << std::endl;

    // // 調用函數計算矩陣的指數
    // // myexpm::expm_higham05(A);

    // // 打印計算後的矩陣
    // std::cout << "矩陣指數 exp(A):\n" << A << std::endl;

    // std::cout << "Eigen version: " << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;

    // // 定义一个旋转向量（角轴表示，向量的方向表示旋转轴，长度表示旋转角度）
    // Eigen::Vector3d omega(0.0, 0.0, M_PI/2.54); // 以 Z 轴为轴，旋转 45 度

    // // 计算指数映射得到旋转矩阵
    // Sophus::SO3d R = Sophus::SO3d::exp(omega);

    // // 输出结果
    // std::cout << "旋转矩阵:\n" << R.matrix() << std::endl;
    //-------------------------------------------------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------------------------------------------------
    // dirac func用法
    // double x = 0.00001;
    // double result = approx_dirac(x);
    // std::cout << "The approximation of the Dirac delta function at x = " << x << " is " << result << std::endl;
    //-------------------------------------------------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------------------------------------------------
    // // 假设矩阵大小为18x18
    // Eigen::MatrixXd xtrue2err(19, 18);

    // // 使用setZero初始化所有元素为0
    // xtrue2err.setZero();

    // // 为特定的元素赋值
    // // 对角线元素
    // for (int i = 0; i < 6; ++i) {
    //     xtrue2err(i, i) = 1.0;
    // }

    // // 特定位置的赋值
    // xtrue2err(6, 6) = 4.60066766197058e-06;
    // xtrue2err(6, 7) = 3.31581966234253e-06;
    // xtrue2err(6, 8) = -3.03217386411181e-08;
    // xtrue2err(7, 6) = 0.499999999967838;
    // xtrue2err(7, 7) = -3.03217386411181e-08;
    // xtrue2err(7, 8) = -3.31581966234253e-06;
    // xtrue2err(8, 6) = 3.03217386411181e-08;
    // xtrue2err(8, 7) = 0.499999999967838;
    // xtrue2err(8, 8) = 4.60066766197058e-06;
    // xtrue2err(8, 6) = 3.31581966234253e-06;
    // xtrue2err(8, 7) = -4.60066766197058e-06;
    // xtrue2err(8, 8) = 0.499999999967838;

    // // Ackerman Vehicle Param
    // double L = 176; // (mm)
    // double L1 = 112;
    // double L2 = 44.52;
    // double L3 = 85;
    // double L4 = L2;
    // double l_rear = 164;
    // double l_ax = (l_rear - L1) / 2;
    // double r_wheel = 100;

    // double Ackermanparam[] = {L, L1, L2, L3, L4, l_rear, l_ax, r_wheel};

    // double sampling_time_ack = 0.01;

    // Eigen::MatrixXd P_fusion = 0.001 * Eigen::MatrixXd::Identity(18, 18);

    // Eigen::MatrixXd V = 0.5 * Eigen::MatrixXd::Identity(2, 2);

    // struct0_T AckermanBaseMea; // 創建結構體變數
    // const struct0_T *ptrToAckermanBaseMea = &AckermanBaseMea; // 創建一個指向myStruct的const指針

    // // 給steer和vel賦值
    // AckermanBaseMea.steer = 0;
    // AckermanBaseMea.vel = 0;

    // double encoder_pri = 0;
    // double theta = 0;


    // double* arrayP = MatrixUtils::EigenMatrixToArray(P_fusion);
    // double* arrayV = MatrixUtils::EigenMatrixToArray(V);
    // double* arrayxtrue2err = MatrixUtils::EigenMatrixToArray(xtrue2err);
    // double R[36];

    // // for(int i = 0; i < 100; i++) {
    // //     std::cout << arrayP[i] << std::endl; // 或者使用其他格式化方式
    // // }


    // MeaCov2C(arrayP, arrayV,
    //         arrayxtrue2err, ptrToAckermanBaseMea,
    //         encoder_pri, theta,
    //         Ackermanparam, sampling_time_ack,
    //         R);
    // // for(int i = 0; i < 36; i++) {
    // //     std::cout << R[i] << std::endl; // 或者使用其他格式化方式
    // // }
    //-------------------------------------------------------------------------------------------------------------------------
    // double bb, cc, aa;
    // double ww = 0.6011, xx = -1.6669 * std::pow(10.0, -13.0), yy = -4.9211 * std::pow(10.0, -12.0), zz = 0.79917;
    // Eigen::Quaterniond qqqq(ww, xx, yy, zz);
    // Eigen::Vector3d vec = MatrixUtils::quaternionToEulerAngles(qqqq);
    // std::cout << "yaw = " << vec[2] << std::endl;
    // std::cout << MatrixUtils::acot(3.0/2.0) << std::endl;
    // std::cout << atan(2.0/3.0) << std::endl;



    // // 繪製圖形
    // plt::figure();
    // // 准备向量起点数据
    // std::vector<double> X = {0, 1, 2};
    // std::vector<double> Y = {0, 1, 2};

    // // 准备向量方向数据
    // std::vector<double> U = {1, -1, 1};
    // std::vector<double> V = {1, 1, -1};

    // // 使用quiver绘制向量
    // plt::quiver(X, Y, U, V);

    // 显示图表
    // plt::show();






    // const double start = 0; // 起始值
    // const double end = 2 * M_PI; // 结束值，2*pi
    // const double step = 0.0001; // 步长
    // std::vector<double> cot_values; // 存储cot(x)的值
    // std::vector<double> cot_input;

    // for(double x = start; x <= end; x += step) {
    //     double cot_x = MatrixUtils::cot(x);
    //     cot_values.push_back(cot_x);
    //     cot_input.push_back(x);
    // }

    // // 仅作示例，打印部分结果
    // std::cout << "Cotangent values at some points:" << std::endl;
    // // for(size_t i = 0; i < 10; ++i) { // 打印前10个值作为示例
    // //     std::cout << "cot(" << i * step << ") = " << cot_values[i] << std::endl;
    // // }
    // plt::figure();
    // plt::plot(cot_input, cot_values);
    // plt::xlim(0.05, 2 * M_PI - 0.05);
    // plt::ylim(-10, 10);
    // plt::show();

    // Eigen::Quaterniond a(1, 2, 3, 4);
    // Eigen::Quaterniond b(2, 3, 4, 5);
    // Eigen::Quaterniond NominalState_q_nominal = a * b;
    // // 打印四元數
    // std::cout << "Quaternion q:" << std::endl;
    // std::cout << "w: " << NominalState_q_nominal.w() << std::endl;
    // std::cout << "x: " << NominalState_q_nominal.x() << std::endl;
    // std::cout << "y: " << NominalState_q_nominal.y() << std::endl;
    // std::cout << "z: " << NominalState_q_nominal.z() << std::endl;

    return 0;
}


// 換回mea// 換回mea// 換回mea// 換回mea// 換回mea// 換回mea// 換回mea// 換回mea// 換回mea