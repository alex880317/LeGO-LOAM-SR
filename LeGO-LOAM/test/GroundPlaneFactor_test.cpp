#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>

// 包含您的自定義因子頭文件
#include "GroundPlaneFactor.h"

int main(int argc, char **argv)
{
    // 定義關鍵字
    gtsam::Key poseKey = gtsam::Symbol('x', 0);

    // 創建初始估計值
    gtsam::Values initialEstimate;
    // 設置一個初始位姿

    // 定義旋轉角度（以弧度為單位）
    double angleX = 10 * M_PI / 180; // 10 度轉換為弧度
    double angleY = 10 * M_PI / 180; // 10 度轉換為弧度
    // 創建繞 X 和 Y 軸的旋轉矩陣
    gtsam::Rot3 R_x = gtsam::Rot3::Rx(angleX);
    gtsam::Rot3 R_y = gtsam::Rot3::Ry(angleY);

    // 注意乘法順序，先應用 Rx，然後應用 Ry
    gtsam::Rot3 R = R_y * R_x;

    gtsam::Pose3 initialPose(R, gtsam::Point3(10, 10, 3.5));
    initialEstimate.insert(poseKey, initialPose);
    // std::cout << "R = " << R << std::endl;
    // std::cout << "R = " << gtsam::Rot3::RzRyRx(0, 10 * M_PI / 180, 10 * M_PI / 180) << std::endl;

    // 創建噪聲模型
    // 這裡假設噪聲為各向同性的，標準差為0.1
    gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);

    // 定義測量的法向量和距離
    // 假設真實的地面法向量為 (0, 0, 1)，距離為 0
    gtsam::Rot3 Rx = gtsam::Rot3::Rx(M_PI / 180 * 3); // X 軸旋轉3度
    gtsam::Rot3 Ry = gtsam::Rot3::Ry(M_PI / 180 * 5); // Y 軸旋轉5度
    gtsam::Point3 measuredNormal(0.0, 0.0, 1.0);

    // 將旋轉應用到measuredNormal上
    gtsam::Point3 transformedNormal = Ry * Rx * measuredNormal;
    double measuredDistance = 0.1;

    // 創建因子圖
    gtsam::NonlinearFactorGraph graph;

    // 創建自定義因子
    // 如果不使用ROS節點，可以傳遞一個空指針
    rclcpp::Node::SharedPtr node = nullptr;
    auto factor = boost::make_shared<GroundPlaneFactor>(
        poseKey, transformedNormal, measuredDistance, noiseModel, node);

    // 將因子添加到因子圖中
    graph.add(factor);

    // 創建ISAM2參數和對象
    gtsam::ISAM2Params parameters;
    gtsam::ISAM2 isam(parameters);

    // 進行優化更新
    isam.update(graph, initialEstimate);

    // 獲取優化後的估計值
    gtsam::Values currentEstimate = isam.calculateEstimate();

    // 提取優化後的位姿
    gtsam::Pose3 optimizedPose = currentEstimate.at<gtsam::Pose3>(poseKey);

    // 輸出結果
    std::cout << "Initial Pose:\n"
              << initialPose << std::endl;
    std::cout << "Optimized Pose:\n"
              << optimizedPose << std::endl;

    return 0;
}
