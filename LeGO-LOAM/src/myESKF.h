#ifndef ESKF_H
#define ESKF_H

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <vector>
#include <cmath>
#include "nlohmann/json.hpp"
#include "MeaCovFromMatlab/MeaCov2C_pkg/MeaCov2C.h"


// typedef struct{
//     double steer;
//     double vel;
// } struct0_T;

class ESKF {
public:
    // 构造函数
    ESKF(const Eigen::MatrixXd& Omega_m, 
         const Eigen::MatrixXd& Acc_m, 
         const Eigen::MatrixXd& GPS, 
         int iter);
    void runESKF();
    
    void loadFromJson(const std::vector<std::string>& filename);
    using json = nlohmann::json;

    // std::vector<std::vector<double>> Acc_GT;
    Eigen::MatrixXd ConvertToEigenMatrix(const std::vector<std::vector<double>>& data);
    std::vector<Eigen::Vector3d> ConvertToEigenVector3d(const std::vector<std::vector<double>>& input);
    Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);


private:
    std::string filename = "/home/iec/colcon_ws/src/LeGO-LOAM-SR/LeGO-LOAM/src/VirtualDataFromMatlab/IMUData.json";

    std::vector<Eigen::Vector3d> Acc_GT;
    std::vector<Eigen::Vector3d> Omega_GT;
    std::vector<Eigen::Vector3d> Acc_mea;
    std::vector<Eigen::Vector3d> Omega_mea;
    std::vector<Eigen::Vector3d> Position_GT;
    std::vector<Eigen::Vector3d> Attitude_GT;
    std::vector<Eigen::Vector3d> Position_mea;
    std::vector<Eigen::Vector3d> Attitude_mea;
    std::vector<Eigen::Vector3d> pos;
    std::vector<Eigen::Vector3d> vel;
    std::vector<Eigen::Vector3d> att;
    std::vector<Eigen::Vector3d> accb;
    std::vector<Eigen::Vector3d> gyrob;
    std::vector<double> vel_count_GT;
    std::vector<double> steer_count_GT;
    std::vector<double> vel_count_mea;
    std::vector<double> steer_count_mea;

    Eigen::MatrixXd K1;
    Eigen::MatrixXd K2;
    Eigen::VectorXd XFusion;

    // 宣告結構變數
    struct KalmanState {
        Eigen::Vector3d Pos;
        Eigen::Vector3d Vel;
        Eigen::Quaterniond Qua;
        Eigen::Vector3d AccBias;
        Eigen::Vector3d GyroBias;
        Eigen::Vector3d Grav;
    };

    KalmanState NominalState;
    KalmanState NominalPropagation;
    KalmanState State;
    KalmanState ErrorState;
    
    struct Ackermanstate {
        Eigen::Vector3d Pos;
        Eigen::Vector3d Vel;
        Eigen::Quaterniond Qua;
    };
    Ackermanstate AckermanState;
    Ackermanstate AckermanPropagation;
    


    // 成员变量
    Eigen::MatrixXd Omega_m;
    Eigen::MatrixXd Acc_m;
    Eigen::MatrixXd GPSData;
    int Iter;
    // double sampling_time;
    double sampling_time = 0.01;
    double sampling_time_slam = 0.1;
    double sampling_time_ack = 0.01;
    int end_time = 50;
    int i_imu_update;
    int i_slam_update;
    int i_ack_update;
    std::vector<double> t;
    std::vector<double> t_slam;
    std::vector<double> t_ack;
    


    Eigen::MatrixXd LidarAttitude_m;
    Eigen::MatrixXd LidarPosition_m;

    double rear_wheel_count = 60000*45/35;
    double heading_angle_count = std::pow(2.0, 14.0);

    // Eigen::VectorXd Vel;
    // Eigen::VectorXd steer;

    int initial_pos;
    int initial_vel;
    int initial_theta;

    Eigen::Vector3d gravity;
    Eigen::Vector3d Acc_b;
    Eigen::Vector3d Gyro_b;

    // Error State Initial Value
    Eigen::Vector3d PosError;Eigen::Vector3d VelError;Eigen::Vector3d QuaError;
    Eigen::Vector3d AccBiasError;Eigen::Vector3d GyroBiasError;Eigen::Vector3d GravError;
    Eigen::VectorXd XError;
    Eigen::VectorXd XErrorPre;
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_pri;
    Eigen::MatrixXd P_fusion;

    Eigen::MatrixXd Fx;
    Eigen::MatrixXd Fi;
    Eigen::MatrixXd Qi;

    float AccSTD = 0.01;float GyroSTD = 0.0015;float AccBiasSTD = 0.0005;float GyroBiasSTD = 0.0005;
    Eigen::VectorXd IMU_noise_scale;

    // Measurement1 Initial Value
    float TranslationSTD = 0.01;float RotationSTD = 1;
    Eigen::VectorXd Lidar_noise_scale;
    std::vector<double> qua_noise;

    Eigen::MatrixXd V1 = Eigen::MatrixXd::Identity(7, 7);
    Eigen::MatrixXd Hx1 = Eigen::MatrixXd::Zero(7, 19);
    Eigen::MatrixXd Xdeltax;
    Eigen::MatrixXd H1 = Eigen::MatrixXd::Zero(7, 18);


    // Measurement2 Initial Value
    // Eigen::VectorXd Ackermanparam;

    double encoder_pri;

    float velSTD = 0.1;float steeringSTD = 0.1;
    Eigen::VectorXd Ackerman_noise_scale;

    Eigen::MatrixXd V2 = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd Hx2 = Eigen::MatrixXd::Zero(6, 19);
    Eigen::MatrixXd xtrue2err = Eigen::MatrixXd::Zero(19, 18);
    Eigen::MatrixXd H2 = Eigen::MatrixXd::Zero(7, 18);

    // Reset Initial Value
    Eigen::MatrixXd G;


    // Lidar
    std::vector<double> roll_GT;
    std::vector<double> pitch_GT;
    std::vector<double> yaw_GT;
    std::vector<double> roll_mea;
    std::vector<double> pitch_mea;
    std::vector<double> yaw_mea;


    // 成员函数声明
    void ValueInitialize();
    void NominalStatePropagation();
    void ErrorStateTransitionMatrix();
    void LidarFusionProcess();
    void LidarMeasurementTransitionMatrix();
    void LidarMeasurementQuaNoiseScale();
    void AckermanFusionProcess();
    void AckermanMeasurementTransitionMatrix();
    // 自定义结构体定义
    // struct TrueState {
    //     Eigen::VectorXd PosTrue;
    //     Eigen::VectorXd VelTrue;
    //     Eigen::Quaterniond QuaTrue;
    //     Eigen::VectorXd AccTrue;
    //     Eigen::VectorXd GyroTrue;
    //     Eigen::VectorXd GravTrue;
    // };
    // 函数声明
    void Injection();

    void ResetTransitionMatrix();

    void AckermanStatePropagation();

    // struct AckermanBaseMea {
    //     double L;
    //     double L1;
    //     double L2;
    //     double L3;
    //     double L4;
    //     double l_rear;
    //     double l_ax;
    //     double r_wheel;
    // };
    struct0_T AckermanBaseMea; // 創建結構體變數
    const struct0_T *ptrToAckermanBaseMea = &AckermanBaseMea; // 創建一個指向myStruct的const指針

    double theta;

    // Ackerman Vehicle Param
    double L = 176; // (mm)
    double L1 = 112;
    double L2 = 44.52;
    double L3 = 85;
    double L4 = L2;
    double l_rear = 164;
    double l_ax = (l_rear - L1) / 2;
    double r_wheel = 100;

    double Ackermanparam[8] = {L, L1, L2, L3, L4, l_rear, l_ax, r_wheel};
    


    // 讀取json相關
    
    std::vector<Eigen::Vector3d> omega_gt;

    // void loadFromJson(const std::string& filename);

    std::vector<Eigen::Vector3d> Fusion_Pose_Data;
    std::vector<Eigen::Vector3d> Fusion_Vel_Data;
    std::vector<Eigen::Vector3d> Fusion_Att_Data;
    std::vector<Eigen::Vector3d> Fusion_AccBias_Data;
    std::vector<Eigen::Vector3d> Fusion_GyroBias_Data;

    // debug
    int d;
    std::vector<double> ack_vel_x;
    std::vector<double> ack_vel_y;
    std::vector<double> ack_vel_z;
    

};

#endif // ESKF_H
