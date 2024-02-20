#ifndef ESKF_H
#define ESKF_H

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <vector>
#include <cmath>
#include "nlohmann/json.hpp"



class ESKF {
public:
    // 构造函数
    ESKF(const Eigen::MatrixXd& Omega_m, 
         const Eigen::MatrixXd& Acc_m, 
         const Eigen::MatrixXd& GPS, 
         int iter, 
         double sampling_time);
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
    std::vector<double>vel_count_GT;
    std::vector<double> steer_count_GT;
    std::vector<double> vel_count_mea;
    std::vector<double> steer_count_mea;

    // 宣告結構變數
    struct State {
        Eigen::Vector3d Pos;
        Eigen::Vector3d Vel;
        Eigen::Quaterniond Qua;
        Eigen::Vector3d AccBias;
        Eigen::Vector3d GyroBias;
        Eigen::Vector3d Grav;
    };

    State NominalState;

    // 成员变量
    Eigen::MatrixXd Omega_m;
    Eigen::MatrixXd Acc_m;
    Eigen::MatrixXd GPSData;
    int Iter;
    // double sampling_time;
    float sampling_time = 0.01;
    float sampling_time_slam = 0.1;
    float sampling_time_ack = 0.01;
    int end_time = 50;


    Eigen::MatrixXd LidarAttitude_m;
    Eigen::MatrixXd LidarPosition_m;

    double rear_wheel_count = 60000*45/35;
    int heading_angle_count = std::pow(2.0, 14.0);

    Eigen::VectorXd Vel;
    Eigen::VectorXd steer;

    int initial_pos;
    int initial_vel;
    int initial_theta;

    Eigen::Vector3d g;
    Eigen::Vector3d Acc_b;
    Eigen::Vector3d Gyro_b;

    // Error State Initial Value
    Eigen::Vector3d PosError;Eigen::Vector3d VelError;Eigen::Vector3d QuaError;
    Eigen::Vector3d AccBiasError;Eigen::Vector3d GyroBiasError;Eigen::Vector3d GravError;
    Eigen::MatrixXd XError;

    Eigen::MatrixXd Fx;
    Eigen::MatrixXd Fi;
    Eigen::MatrixXd Qi;

    float AccSTD = 0.01;float GyroSTD = 0.0015;float AccBiasSTD = 0.0005;float GyroBiasSTD = 0.0005;
    Eigen::VectorXd IMU_noise_scale;

    // Measurement1 Initial Value
    float TranslationSTD = 0.01;float RotationSTD = 0.0015;
    Eigen::VectorXd Lidar_noise_scale;

    Eigen::MatrixXd V1;
    Eigen::MatrixXd Hx1;
    Eigen::MatrixXd Xdeltax;

    // Measurement2 Initial Value
    Eigen::VectorXd Ackermanparam;

    double encoder_pri = 0.0;

    float velSTD = 0.1;float steeringSTD = 0.1;
    Eigen::VectorXd Ackerman_noise_scale;

    Eigen::MatrixXd V2;
    Eigen::MatrixXd Hx2;

    // Reset Initial Value
    Eigen::MatrixXd G;

    
    


    // 成员函数声明
    void ValueInitialize();
    void NominalStatePropagation();
    void ErrorStateTransitionMatrix();
    void LidarMeasurementTransitionMatrix(const std::vector<Eigen::MatrixXd>& NominalState, 
                                          Eigen::MatrixXd& Hx, 
                                          Eigen::MatrixXd& V, 
                                          Eigen::MatrixXd& Xdeltax, 
                                          const Eigen::Vector2d& noise_scale,
                                          const Eigen::Vector4d& qua_noise);
    void AckermanMeasurementTransitionMatrix(const std::vector<Eigen::MatrixXd>& NominalState, 
                                             Eigen::MatrixXd& Hx, 
                                             Eigen::MatrixXd& V, 
                                             Eigen::MatrixXd& Xdeltax, 
                                             const Eigen::Vector2d& noise_scale);
    // 自定义结构体定义
    struct TrueState {
        Eigen::VectorXd PosTrue;
        Eigen::VectorXd VelTrue;
        Eigen::Quaterniond QuaTrue;
        Eigen::VectorXd AccTrue;
        Eigen::VectorXd GyroTrue;
        Eigen::VectorXd GravTrue;
    };
    // 函数声明
    void Injection(const std::vector<Eigen::MatrixXd>& NominalState,
                   const Eigen::VectorXd& XFusion);

    void ResetTransitionMatrix();

    


    // 讀取json相關
    
    std::vector<Eigen::Vector3d> omega_gt;

    // void loadFromJson(const std::string& filename);
    

};

#endif // ESKF_H
