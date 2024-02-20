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
// #include "rclcpp/rclcpp.hpp"
// using namespace boost::numeric::ublas;

ESKF::ESKF(const Eigen::MatrixXd& Omega_m, const Eigen::MatrixXd& Acc_m, const Eigen::MatrixXd& GPS, int iter, double sampling_time)
    : Omega_m(Omega_m), Acc_m(Acc_m), GPSData(GPS), Iter(iter), sampling_time(sampling_time) {}

void ESKF::ValueInitialize() {
    // 處理姿態從尤拉角轉成四元數
    Eigen::Quaterniond qua = eulerToQuaternion(att[0][0], att[0][1], att[0][2]);
    std::cout << "Quaternion: [" << qua.w() << ", " << qua.x() << ", " << qua.y() << ", " << qua.z() << "]" << std::endl;

    // NominalStatePropagation
    NominalState.Pos = pos[0];
    NominalState.Vel = vel[0];
    NominalState.Qua = qua;
    NominalState.AccBias = accb[0];
    NominalState.GyroBias = gyrob[0];
    NominalState.Grav = pos[0];

    // ErrorStateTransitionMatrix
    Fx = Eigen::MatrixXd::Identity(18, 18);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
    Fx.block<3,3>(0, 3) = I * sampling_time;
    Fx.block<3,3>(3, 15) = I * sampling_time;
    Fx.block<3,3>(6, 12) = -I * sampling_time;


}

void ESKF::NominalStatePropagation() {
    int i = Iter;
    Eigen::Vector3d p = NominalState.Pos;
    Eigen::Vector3d v = NominalState.Vel;
    Eigen::Quaterniond q = NominalState.Qua;
    Eigen::Vector3d Acc_b = NominalState.AccBias;
    Eigen::Vector3d Gyro_b = NominalState.GyroBias;
    Eigen::Vector3d g = NominalState.Grav;

    double delta_t = sampling_time;

    Eigen::MatrixXd R = q.toRotationMatrix();   //用的是eigen的func，可換成自己寫的
    Eigen::MatrixXd NominalState_p_nominal = p + v * delta_t + 0.5 * (R * (Acc_mea[i] - Acc_b) + g) * (delta_t * delta_t);
    Eigen::MatrixXd NominalState_v_nominal = v + (R * (Acc_mea[i] - Acc_b) + g) * delta_t;

    Eigen::Vector3d omega = Omega_mea[i] - Gyro_b;
    double omega_norm = omega.norm();
    Eigen::Quaterniond RotateIncrement;
    if (omega_norm == 0) {
        RotateIncrement = Eigen::Quaterniond(1, 0, 0, 0);
    } else {
        RotateIncrement = Eigen::Quaterniond(Eigen::AngleAxisd(omega_norm * delta_t * 0.5, omega.normalized()));
    }

    Eigen::Quaterniond NominalState_q_nominal = q * RotateIncrement;
    NominalState_q_nominal.normalize(); // 確保結果是單位四元數

    // 封裝結果
    // std::vector<Eigen::Vector3d> NominalState; // 根據需要調整大小
    // NominalState << NominalState_p_nominal, NominalState_v_nominal, Eigen::Vector4d(NominalState_q_nominal.w(), NominalState_q_nominal.x(), NominalState_q_nominal.y(), NominalState_q_nominal.z()), Acc_b, Gyro_b, g;
    NominalState.Pos = p;
    NominalState.Vel = v;
    NominalState.Qua = q;
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
// void ESKF::ErrorStateTransitionMatrix() {
//     int i = Iter;
//     double delta_t = sampling_time;

//     Eigen::Vector3d p = NominalState.Pos;
//     Eigen::Vector3d v = NominalState.Vel;
//     Eigen::Quaterniond q = NominalState.Qua;
//     Eigen::Vector3d Acc_b = NominalState.AccBias;
//     Eigen::Vector3d Gyro_b = NominalState.GyroBias;
//     Eigen::Vector3d g = NominalState.Grav;

//     Eigen::MatrixXd R = q.toRotationMatrix();

//     double scale_acc_noise = IMU_noise_scale(0);
//     double scale_gyro_noise = IMU_noise_scale(1);
//     double scale_acc_bias_noise = IMU_noise_scale(2);
//     double scale_gyro_bias_noise = IMU_noise_scale(3);

//     Eigen::MatrixXd sigma_vel = Eigen::MatrixXd::Identity(3, 3) * scale_acc_noise;
//     Eigen::MatrixXd sigma_theta = Eigen::MatrixXd::Identity(3, 3) * scale_gyro_noise;
//     Eigen::MatrixXd sigma_acc = Eigen::MatrixXd::Identity(3, 3) * scale_acc_bias_noise;
//     Eigen::MatrixXd sigma_omega = Eigen::MatrixXd::Identity(3, 3) * scale_gyro_bias_noise;

//     // 更新 Fx 和 Qi
//     // 注意：在 C++ 中，矩陣索引是從 0 開始的
//     Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
//     Fx.block<3, 3>(3, 9) = -R * delta_t;
//     Fx.block<3, 3>(3, 6) = -R * MatrixUtils::vectorToCrossMatrix(Acc_mea[i] - Acc_b) * delta_t;
//     Fx.block<3, 3>(6, 6) = (Eigen::AngleAxisd(-omega_norm * delta_t, omega.normalized())).toRotationMatrix();

//     Qi.block<3, 3>(0, 0) = sigma_vel * sigma_vel * delta_t * delta_t;
//     Qi.block<3, 3>(3, 3) = sigma_theta * sigma_theta * delta_t * delta_t;
//     Qi.block<3, 3>(6, 6) = sigma_acc * sigma_acc * delta_t;
//     Qi.block<3, 3>(9, 9) = sigma_omega * sigma_omega * delta_t;

//     // 不需要返回值，因為 Fx 和 Qi 是通過引用傳遞的
// }

// // void ESKF::GPSMeasurementTransitionMatrix(Eigen::MatrixXd& Hx, Eigen::MatrixXd& V, Eigen::MatrixXd& Xdeltax, const NominalState& Nominal_state, const Eigen::VectorXd& noise_scale, const Eigen::VectorXd& qua_noise) {
// //     Eigen::Quaterniond q_nominal = Nominal_state.q_nominal;
    
// //     Eigen::MatrixXd Qdeltatheta(4, 3);
// //     Qdeltatheta << -q_nominal.x(), -q_nominal.y(), -q_nominal.z(),
// //                     q_nominal.w(), -q_nominal.z(), q_nominal.y(),
// //                     q_nominal.z(), q_nominal.w(), -q_nominal.x(),
// //                    -q_nominal.y(), q_nominal.x(), q_nominal.w();
// //     Xdeltax.block<4, 3>(6, 6) = Qdeltatheta;

// //     double scale_translation_noise = noise_scale(0);
// //     double scale_rotation_noise = noise_scale(1);

// //     Eigen::MatrixXd sigma_translation = Eigen::MatrixXd::Identity(3, 3) * scale_translation_noise;
// //     // Eigen::MatrixXd sigma_rotation = Eigen::MatrixXd::Identity(4, 4) * scale_rotation_noise;
// //     // 根據需求啟用或修改上面的 sigma_rotation 計算

// //     V.block<3, 3>(0, 0) = sigma_translation.cwiseProduct(sigma_translation);
// //     // V.block<4, 4>(3, 3) = sigma_rotation.cwiseProduct(sigma_rotation);

// //     Hx = Hx * Xdeltax;
// //     // 不需要返回值，因為 Hx 和 V 是通過引用傳遞的
// // }

// void ESKF::LidarMeasurementTransitionMatrix(const std::vector<Eigen::MatrixXd>& NominalState, 
//                                           Eigen::MatrixXd& Hx, 
//                                           Eigen::MatrixXd& V, 
//                                           Eigen::MatrixXd& Xdeltax, 
//                                           const Eigen::Vector2d& noise_scale,
//                                           const Eigen::Vector4d& qua_noise) {
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
//     Eigen::MatrixXd sigma_rotation = Eigen::MatrixXd::Identity(4, 4);
//     for (int i = 0; i < 4; ++i) {
//         sigma_rotation(i, i) = scale_rotation_noise * qua_noise(i);
//     }

//     V.block<3, 3>(0, 0) = sigma_translation.cwiseProduct(sigma_translation);
//     V.block<4, 4>(3, 3) = sigma_rotation.cwiseProduct(sigma_rotation);

//     Hx = Hx * Xdeltax;
//     // 不需要返回值，因為 Hx 和 V 是通過引用傳遞的
// }

// void ESKF::AckermanMeasurementTransitionMatrix(const std::vector<Eigen::MatrixXd>& NominalState, 
//                                              Eigen::MatrixXd& Hx, 
//                                              Eigen::MatrixXd& V, 
//                                              Eigen::MatrixXd& Xdeltax, 
//                                              const Eigen::Vector2d& noise_scale) {
//     Eigen::Quaterniond q_nominal = Nominal_state.q_nominal;

//     Eigen::MatrixXd Qdeltatheta(4, 3);
//     Qdeltatheta << -q_nominal.x(), -q_nominal.y(), -q_nominal.z(),
//                     q_nominal.w(), -q_nominal.z(), q_nominal.y(),
//                     q_nominal.z(), q_nominal.w(), -q_nominal.x(),
//                    -q_nominal.y(), q_nominal.x(), q_nominal.w();
//     Xdeltax.block<4, 3>(6, 6) = Qdeltatheta;

//     double scale_vel_noise = noise_scale(0);
//     double scale_steering_noise = noise_scale(1);

//     Eigen::MatrixXd sigma_vel = Eigen::MatrixXd::Identity(4, 4) * scale_vel_noise * 0.01; // 0.01 為積分的 delta_t
//     Eigen::MatrixXd sigma_attitude = Eigen::MatrixXd::Identity(4, 4) * scale_steering_noise;

//     V.block<4, 4>(0, 0) = sigma_vel.cwiseProduct(sigma_vel);
//     V.block<4, 4>(4, 4) = sigma_attitude.cwiseProduct(sigma_attitude);

//     Hx = Hx * Xdeltax;
//     // 不需要返回值，因為 Hx 和 V 是通過引用傳遞的
// }

// void ESKF::Injection(const std::vector<Eigen::MatrixXd>& NominalState,
//                      const Eigen::VectorXd& XFusion) {
//     Nominal_state.p_nominal += Error_state.XFusion.block<3, 1>(0, 0);
//     Nominal_state.v_nominal += Error_state.XFusion.block<3, 1>(3, 0);

//     Eigen::Vector3d delta_theta = Error_state.XFusion.block<3, 1>(6, 0);
//     if (delta_theta.norm() == 0) {
//         // 不需要改變 q_nominal
//     } else {
//         Eigen::Quaterniond delta_q(Eigen::AngleAxisd(delta_theta.norm(), delta_theta.normalized()));
//         Nominal_state.q_nominal = Nominal_state.q_nominal * delta_q;
//     }

//     Nominal_state.AccBiasNominal += Error_state.XFusion.block<3, 1>(9, 0);
//     Nominal_state.GyroBiasNominal += Error_state.XFusion.block<3, 1>(12, 0);
//     Nominal_state.GravNominal += Error_state.XFusion.block<3, 1>(15, 0);
// }

// void ESKF::ResetTransitionMatrix() {
//     Eigen::Matrix3d G_7x9 = Eigen::Matrix3d::Identity() - MatrixUtils::vectorToCrossMatrix(0.5 * Error_state.XFusion.block<3, 1>(6, 0));
//     // 或者，如果您需要使用 expm 函數，您可能需要自己實現它，因為 Eigen 沒有直接提供
//     // Eigen::Matrix3d G_7x9 = expm(-MatrixUtils::vectorToCrossMatrix(0.5 * Error_state.XFusion.block<3, 1>(6, 0)));

//     G.block<3, 3>(6, 6) = G_7x9;

//     P = G * P * G.transpose();
// }

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
    std::vector<std::vector<double>>Acc_GT_data = IMUData["IMUData"]["Acc_GT"].get<std::vector<std::vector<double>>>();
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
    std::vector<double> vel_count_GT = EncoderData["EncoderData"]["vel_count_GT"].get<std::vector<double>>();
    std::vector<double> steer_count_GT = EncoderData["EncoderData"]["steer_count_GT"].get<std::vector<double>>();
    std::vector<double> vel_count_mea = EncoderData["EncoderData"]["vel_count_mea"].get<std::vector<double>>();
    std::vector<double> steer_count_mea = EncoderData["EncoderData"]["steer_count_mea"].get<std::vector<double>>();
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

    std::cout << Acc_GT_data[0][0] << std::endl;

    if (!Acc_GT_data.empty()) {
        std::cout << "Acc_GT 第一组向量: ";
        for (double value : Acc_GT_data[0]) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Acc_GT 是空的或未正确加载数据。" << std::endl;
    }

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

    // std::cout << att[0] << std::endl;
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
    std::cout << roll << std::endl;
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

void ESKF::runESKF() {
    // 參數設置
    // double sampling_time = 0.01;
    // double end_time = 50;
    // 更多參數...

    // 加載數據
    // 這裡需要替換為 C++ 中讀取數據的代碼
    // 例如使用文件流讀取數據

    // 初始化
    // Eigen::VectorXd initial_pos = pos_GT.col(0);
    // Eigen::VectorXd initial_vel = vel_GT.col(0);
    // Eigen::VectorXd p = initial_pos;
    // Eigen::VectorXd v = initial_vel;
    // 更多初始化...
    
    // 處理循環
    ValueInitialize();
    NominalStatePropagation();
    // for (size_t i = 0; i < end_time / sampling_time - 1; ++i) {
    //     // 每次迭代的處理
    //     // 例如：Nominal_state = NominalStatePropagation(state);
    //     NominalStatePropagation(NominalState);
    //     // 根據您的 ESKF 類的具體實現調用相應的方法
    // }
}

int main(){
    Eigen::MatrixXd Omega_m; // 初始化 Omega_m
    Eigen::MatrixXd Acc_m;   // 初始化 Acc_m
    Eigen::MatrixXd GPS;     // 初始化 GPS
    int iter = 0;            // 初始化 iter
    double sampling_time = 0.01; // 設置採樣時間
    ESKF eskf(Omega_m, Acc_m, GPS, iter, sampling_time);

    std::cout << "123" << std::endl;

    // std::string filename = "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/IMUData.json";
    std::vector<std::string> filename = {
        "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/IMUData.json",
        "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/LidarData.json",
        "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/EncoderData.json",
        "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/GroundTruthData.json"
    };


    eskf.loadFromJson(filename);

    eskf.runESKF();


    boost::numeric::ublas::bmatrix A(3, 3);
    A(0,0) = 1; A(0,1) = 2; A(0,2) = 3;
    A(1,0) = 4; A(1,1) = 5; A(1,2) = 6;
    A(2,0) = 7; A(2,1) = 8; A(2,2) = 9;

    // 打印原始矩陣
    std::cout << "原始矩陣 A:\n" << A << std::endl;

    // 調用函數計算矩陣的指數
    myexpm::expm_higham05(A);

    // 打印計算後的矩陣
    std::cout << "矩陣指數 exp(A):\n" << A << std::endl;

    return 0;
}