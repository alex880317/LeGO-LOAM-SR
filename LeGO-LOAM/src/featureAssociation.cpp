// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.

#include "featureAssociation.h"
#include "rclcpp/rclcpp.hpp"
#include <limits> // Alex for std::numeric_limits<float>::quiet_NaN()


const std::string PARAM_VERTICAL_SCANS = "laser.num_vertical_scans";
const std::string PARAM_HORIZONTAL_SCANS = "laser.num_horizontal_scans";
const std::string PARAM_SCAN_PERIOD = "laser.scan_period";
const std::string PARAM_USE_IMU_UNDISTORTION = "laser.use_imu_undistortion";
const std::string PARAM_FREQ_DIVIDER = "mapping.mapping_frequency_divider";
const std::string PARAM_EDGE_THRESHOLD = "featureAssociation.edge_threshold";
const std::string PARAM_SURF_THRESHOLD = "featureAssociation.surf_threshold";
const std::string PARAM_DISTANCE = "featureAssociation.nearest_feature_search_distance";
const std::string PARAM_ANGLE_BOTTOM = "laser.vertical_angle_bottom";
const std::string PARAM_ANGLE_TOP = "laser.vertical_angle_top";
const std::string PARAM_DBFr = "laser.DBFr";
const std::string PARAM_RatioXY = "laser.RatioXY";
const std::string PARAM_RatioZ = "laser.RatioZ";

const float RAD2DEG = 180.0 / M_PI;

FeatureAssociation::FeatureAssociation(const std::string &name, Channel<ProjectionOut> &input_channel,
                                       Channel<AssociationOut> &output_channel)
    : Node(name), _input_channel(input_channel), _output_channel(output_channel) {

  _sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_type", 50, std::bind(&FeatureAssociation::imuHandler, this, std::placeholders::_1));

  // _sub_visual_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     "/visual_cloud", 1000, std::bind(&FeatureAssociation::visualcloudHandler, this, std::placeholders::_1));

  _sub_odom_gt = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 100, std::bind(&FeatureAssociation::odomCallback, this, std::placeholders::_1)); //Alex

  _sub_Ack_encoder = this->create_subscription<sensor_msgs::msg::JointState>(
      "/racebot/joint_states", 100, std::bind(&FeatureAssociation::jointStateCallback, this, std::placeholders::_1)); //Alex

  pubRotateMsgs = this->create_publisher<geometry_msgs::msg::Vector3>("/rotate_msg", 10); //Alex
  
  pubCornerPointsSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_sharp", 1);
  pubCornerPointsLessSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_less_sharp", 1);
  pubSurfPointsFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_flat", 1);
  pubSurfPointsLessFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_less_flat", 1);
  pubSegmentedCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud", 1);
  pubDistortedCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/distorted_cloud", 1);

  _pub_cloud_corner_last = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_corner_last", 2);
  _pub_cloud_surf_last = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf_last", 2);
  _pub_outlier_cloudLast = this->create_publisher<sensor_msgs::msg::PointCloud2>("/outlier_cloud_last", 2);
  pubLaserOdometry = this->create_publisher<nav_msgs::msg::Odometry>("/laser_odom_to_init", 5);

  // _pub_visual_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visual_cloud_last", 1);

  tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  _cycle_count = 0;

  // Declare parameters
  this->declare_parameter(PARAM_VERTICAL_SCANS);
  this->declare_parameter(PARAM_HORIZONTAL_SCANS);
  this->declare_parameter(PARAM_SCAN_PERIOD);
  this->declare_parameter(PARAM_USE_IMU_UNDISTORTION);
  this->declare_parameter(PARAM_FREQ_DIVIDER);
  this->declare_parameter(PARAM_EDGE_THRESHOLD);
  this->declare_parameter(PARAM_SURF_THRESHOLD);
  this->declare_parameter(PARAM_DISTANCE);
  this->declare_parameter(PARAM_ANGLE_BOTTOM);
  this->declare_parameter(PARAM_ANGLE_TOP);
  this->declare_parameter(PARAM_DBFr);
  this->declare_parameter(PARAM_RatioXY);
  this->declare_parameter(PARAM_RatioZ);

  float nearest_dist;

  // Read parameters
  if (!this->get_parameter(PARAM_VERTICAL_SCANS, _vertical_scans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_VERTICAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_HORIZONTAL_SCANS, _horizontal_scans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_HORIZONTAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_SCAN_PERIOD, _scan_period)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SCAN_PERIOD.c_str());
  }
  if (!this->get_parameter(PARAM_USE_IMU_UNDISTORTION, use_imu_undistortion)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_USE_IMU_UNDISTORTION.c_str());
  }
  if (!this->get_parameter(PARAM_FREQ_DIVIDER, _mapping_frequency_div)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_FREQ_DIVIDER.c_str());
  }
  if (!this->get_parameter(PARAM_EDGE_THRESHOLD, _edge_threshold)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_EDGE_THRESHOLD.c_str());
  }
  if (!this->get_parameter(PARAM_SURF_THRESHOLD, _surf_threshold)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SURF_THRESHOLD.c_str());
  }
  if (!this->get_parameter(PARAM_DISTANCE, nearest_dist)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_DISTANCE.c_str());
  }
  if (!this->get_parameter(PARAM_ANGLE_BOTTOM, _ang_bottom)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ANGLE_BOTTOM.c_str());
  }
  if (!this->get_parameter(PARAM_ANGLE_TOP, vertical_angle_top)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ANGLE_TOP.c_str());
  }
  if (!this->get_parameter(PARAM_DBFr, DBFr)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_DBFr.c_str());
  }
  if (!this->get_parameter(PARAM_RatioXY, RatioXY)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_RatioXY.c_str());
  }
  if (!this->get_parameter(PARAM_RatioZ, RatioZ)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_RatioZ.c_str());
  }

  _nearest_feature_dist_sqr = nearest_dist*nearest_dist;
  _ang_resolution_X = (M_PI*2) / (_horizontal_scans);
  _ang_resolution_Y = DEG_TO_RAD*(vertical_angle_top - _ang_bottom) / float(_vertical_scans-1);

  initializationValue();

 _run_thread = std::thread (&FeatureAssociation::runFeatureAssociation, this);
}

FeatureAssociation::~FeatureAssociation()
{
  _input_channel.send({});
  _run_thread.join();
}

void FeatureAssociation::initializationValue() {
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  cloudSmoothness.resize(cloud_size);

  downSizeFilter.setLeafSize(0.2, 0.2, 0.2); //0.2

  segmentedCloud.reset(new pcl::PointCloud<PointType>());
  visualCloud.reset(new pcl::PointCloud<PointType>());
  distortedCloud.reset(new pcl::PointCloud<PointType>());
  outlierCloud.reset(new pcl::PointCloud<PointType>());
  // fullCloud.reset(new pcl::PointCloud<PointType>());
  virtualshadowCloud.reset(new pcl::PointCloud<PointType>()); // Alex

  cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
  cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
  surfPointsFlat.reset(new pcl::PointCloud<PointType>());
  surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

  surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
  surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

  cloudCurvature.resize(cloud_size);
  cloudNeighborPicked.resize(cloud_size);
  cloudNeighborPickedPlane.resize(cloud_size);
  cloudLabel.resize(cloud_size);

  pointSearchCornerInd1.resize(cloud_size);
  pointSearchCornerInd2.resize(cloud_size);

  pointSearchSurfInd1.resize(cloud_size);
  pointSearchSurfInd2.resize(cloud_size);
  pointSearchSurfInd3.resize(cloud_size);

  skipFrameNum = 1;

  for (int i = 0; i < 6; ++i) {
    transformCur[i] = 0;
    transformSum[i] = 0;
  }

  systemInitedLM = false;

  laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
  laserCloudCornerScan.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfScan.reset(new pcl::PointCloud<PointType>());
  laserCloudOri.reset(new pcl::PointCloud<PointType>());
  coeffSel.reset(new pcl::PointCloud<PointType>());
  virtualshadowCloud.reset(new pcl::PointCloud<PointType>);

  laserOdometry.header.frame_id = "camera_init";
  laserOdometry.child_frame_id = "laser_odom";

  laserOdometryTrans.header.frame_id = "camera_init";
  laserOdometryTrans.child_frame_id = "laser_odom";

  isDegenerate = false;

  frameCount = skipFrameNum;


  timeScanCur = 0;
  timeNewSegmentedCloud = 0;
  timeNewSegmentedCloudInfo = 0;
  timeNewOutlierCloud = 0;

  newSegmentedCloud = false;
  newSegmentedCloudInfo = false;
  newOutlierCloud = false;

  imuPointerFront = 0;
  imuPointerLast = -1;
  imuPointerLastIteration = 0;

  imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
  cosImuRollStart = 0; cosImuPitchStart = 0; cosImuYawStart = 0;
  sinImuRollStart = 0; sinImuPitchStart = 0; sinImuYawStart = 0;
  imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;

  imuVeloXStart = 0; imuVeloYStart = 0; imuVeloZStart = 0;
  imuShiftXStart = 0; imuShiftYStart = 0; imuShiftZStart = 0;

  imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
  imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

  imuShiftFromStartXCur = 0; imuShiftFromStartYCur = 0; imuShiftFromStartZCur = 0;
  imuVeloFromStartXCur = 0; imuVeloFromStartYCur = 0; imuVeloFromStartZCur = 0;

  imuAngularRotationXCur = 0; imuAngularRotationYCur = 0; imuAngularRotationZCur = 0;
  imuAngularRotationXLast = 0; imuAngularRotationYLast = 0; imuAngularRotationZLast = 0;
  imuAngularFromStartX = 0; imuAngularFromStartY = 0; imuAngularFromStartZ = 0;

  imuRollPreSurf = 0; imuPitchPreSurf = 0; imuYawPreSurf = 0;
  imuRollPreCorner = 0; imuPitchPreCorner = 0; imuYawPreCorner = 0;
  for (int i = 0; i < imuQueLength; ++i)
  {
    imuTime[i] = 0;
    imuRoll[i] = 0; imuPitch[i] = 0; imuYaw[i] = 0;
    imuAccX[i] = 0; imuAccY[i] = 0; imuAccZ[i] = 0;
    imuVeloX[i] = 0; imuVeloY[i] = 0; imuVeloZ[i] = 0;
    imuShiftX[i] = 0; imuShiftY[i] = 0; imuShiftZ[i] = 0;
    imuAngularVeloX[i] = 0; imuAngularVeloY[i] = 0; imuAngularVeloZ[i] = 0;
    imuAngularRotationX[i] = 0; imuAngularRotationY[i] = 0; imuAngularRotationZ[i] = 0;
  }

  imuRollLast = 0; imuPitchLast = 0; imuYawLast = 0;
  imuShiftFromStartX = 0; imuShiftFromStartY = 0; imuShiftFromStartZ = 0;
  imuVeloFromStartX = 0; imuVeloFromStartY = 0; imuVeloFromStartZ = 0;
  
  //Alex
  odomcall_roll = 0;odomcall_pitch = 0;odomcall_yaw = 0;
  odomInitialIter = 0;
  x_initial = 0;y_initial = 0;z_initial = 0;
  odomStartPosX = 0;odomStartPosY = 0;odomStartPosZ = 0;
  odomStartRoll = 0;odomStartPitch = 0;odomStartYaw = 0;
  odomPointerLast = -1;
  for (int i = 0; i < odomQueLength; ++i)
  {
    odomTime[i] = 0;
    odomRoll[i] = 0; odomPitch[i] = 0; odomYaw[i] = 0;
    odomPosX[i] = 0; odomPosY[i] = 0; odomPosZ[i] = 0;
  }


  for (int i = 0; i < encoderQueLength; ++i)
  {
    encoderTime[i] = 0;
    TurnRF[i] = 0;
    WheelRR[i] = 0;
    WheelLR[i] = 0;
    Wheelk[i] = 0;
  }

  lidar_to_body_centor << 0.008, 0.0, -0.035;
  row_size = 16;col_size = 10;
  row_angle = 0;col_angle = 0;
  row_x = 0;col_y = 0;


}

void FeatureAssociation::visualcloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
  visualCloud->clear();
  pcl::fromROSMsg(*laserCloudMsg, *visualCloud);
  // std::vector<int> indices;
  // pcl::removeNaNFromPointCloud(*_laser_cloud_input, *_laser_cloud_input, indices);
}

void FeatureAssociation::imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr imuIn)
{
    double roll, pitch, yaw;
    tf2::Quaternion orientation;
    tf2::convert(imuIn->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
    float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
    float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;
   
    // std::cout << accX << "," << accY << "," << accZ << std::endl;

    imuPointerLast = (imuPointerLast + 1) % imuQueLength;

    imuTime[imuPointerLast] = imuIn->header.stamp.sec + imuIn->header.stamp.nanosec/1e9;
    //  std::cout << imuTime[imuPointerLast] - imuTime[imuPointerLast-1] << std::endl;
    // std::cout << "Roll: " << roll << std::endl;
    // std::cout << "pitch: " << pitch << std::endl;
    // std::cout << "yaw: " << yaw << std::endl;



    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
    imuYaw[imuPointerLast] = yaw;

    imuAccX[imuPointerLast] = accX;
    imuAccY[imuPointerLast] = accY;
    imuAccZ[imuPointerLast] = accZ;

    imuAngularVeloX[imuPointerLast] = imuIn->angular_velocity.x;
    imuAngularVeloY[imuPointerLast] = imuIn->angular_velocity.y;
    imuAngularVeloZ[imuPointerLast] = imuIn->angular_velocity.z;

    AccumulateIMUShiftAndRotation();
}

//Alex
void FeatureAssociation::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
  // 在這裡處理接收到的 /odom 消息
  // RCLCPP_INFO(this->get_logger(), "Received odom: '%s'", msg->header.frame_id.c_str());
  // 你可以在這裡添加更多的邏輯來處理消息
  
  tf2::Quaternion orientation;
  tf2::convert(msg->pose.pose.orientation, orientation);
  tf2::Matrix3x3(orientation).getRPY(odomcall_roll, odomcall_pitch, odomcall_yaw);
  // RCLCPP_INFO(this->get_logger(), "Ground Truth Roll: '%f'", roll);
  // auto orientation = msg->pose.pose.orientation;
  // RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
  //             orientation.x, orientation.y, orientation.z, orientation.w);
  
  if(odomInitialIter == 0){
    x_initial = msg->pose.pose.position.x;
    y_initial = msg->pose.pose.position.y;
    z_initial = msg->pose.pose.position.z;
    odomInitialIter += 1;
  }

  odomPointerLast = (odomPointerLast + 1) % odomQueLength;
  odomTime[odomPointerLast] = msg->header.stamp.sec + msg->header.stamp.nanosec/1e9;
  
  odomRoll[odomPointerLast] = odomcall_roll;
  odomPitch[odomPointerLast] = odomcall_pitch;
  odomYaw[odomPointerLast] = odomcall_yaw;
  odomPosX[odomPointerLast] = msg->pose.pose.position.x - x_initial;
  odomPosY[odomPointerLast] = msg->pose.pose.position.y - y_initial;
  odomPosZ[odomPointerLast] = msg->pose.pose.position.z - z_initial;
  
  // std::cout << "odomPosX:" << odomPosX[odomPointerLast] << std::endl;
  // std::cout << "odomPosY:" << odomPosY[odomPointerLast] << std::endl;
  // std::cout << "odomPosZ:" << odomPosZ[odomPointerLast] << std::endl;
  // RCLCPP_INFO(this->get_logger(), "odomPosX: '%d'", odomPosX[odomPointerLast]);
  // 生成虛擬點雲
  // GenerateShadowPoint();
}

//Alex
void FeatureAssociation::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // 处理接收到的消息，比如打印信息或其他逻辑
  // RCLCPP_INFO(this->get_logger(), "Received joint state message");
  // 你可能想访问并使用关节状态信息
  // 例如: msg->position, msg->velocity 等
  // 检查是否有位置数据
  if(!msg->position.empty()) {
      // 提取position中的每个值
      for (size_t i = 0; i < msg->position.size(); ++i) {
          double value = msg->position[i];
          // 现在可以使用value进行计算或其他操作
          // RCLCPP_INFO(this->get_logger(), "Position[%zu]: %f", i, value);
      }
  } else {
      // RCLCPP_INFO(this->get_logger(), "No position data available.");
  }
}

void FeatureAssociation::GenerateShadowPoint(){
  // Ground Truth
  virtualshadowCloud->clear();
  row_angle = (atan2(0.120, 0.05) * 2) / (row_size - 1);
  col_angle = (atan2(0.077, 0.05) * 2) / (col_size - 1);
  // pcl::PointCloud<pcl::PointXYZI>::Ptr virtualshadowCloud(new pcl::PointCloud<pcl::PointXYZI>);


  for (int row = 0; row < row_size; row++) {
    row_x = 0.05 * tan((((row_size - 1.0) / 2.0) * row_angle) - (row * row_angle));
    // std::cout << "row_x : " << row_x << std::endl;
    for(int col = 0; col < col_size; col++){
      col_y = 0.05 * tan((((col_size - 1.0) / 2.0) * col_angle) - (col * col_angle));
      virtualshadowpoint.x = col_y + lidar_to_body_centor[1];
      virtualshadowpoint.y = -(0.035f + 0.05f) + lidar_to_body_centor[2];
      virtualshadowpoint.z = row_x + lidar_to_body_centor[0];
      // virtualshadowpoint.intensity = std::numeric_limits<float>::quiet_NaN();
      virtualshadowpoint.intensity = (float)row + (float)17 + (float)col / 10000.0;
      virtualshadowCloud->push_back(virtualshadowpoint);
    }
  }

  // 打印所有点
  // for (const auto& point : *virtualshadowCloud) {
  //     std::cout << "x: " << point.x
  //               << ", y: " << point.y
  //               << ", z: " << point.z
  //               << ", intensity: " << point.intensity << std::endl;
  // }
  // int shadow_i;
  // for (const auto& point : *virtualshadowCloud) {
  //     surfPointsFlat->push_back(point);
  //     std::cout << "virtualshadowCloud: x=" << point.x << ", y=" << point.y << ", z=" << point.z << ", intensity=" << point.intensity << std::endl;
  //     std::cout << "shadow iter : " << shadow_i << std::endl;
  //     shadow_i++;
  // }
  // ESKF

}

void FeatureAssociation::AccumulateIMUShiftAndRotation()
{
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

  float x1 = cos(roll) * accX - sin(roll) * accY; // Alex (rotation matrix z-x-y)
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;

  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (timeDiff < _scan_period) {

    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;

    imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
    imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
    imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
  }
}

void FeatureAssociation::updateImuRollPitchYawStartSinCos(){
    cosImuRollStart = cos(imuRollStart);
    cosImuPitchStart = cos(imuPitchStart);
    cosImuYawStart = cos(imuYawStart);
    sinImuRollStart = sin(imuRollStart);
    sinImuPitchStart = sin(imuPitchStart);
    sinImuYawStart = sin(imuYawStart);
}

void FeatureAssociation::ShiftToStartIMU(float pointTime)
{
    imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
    imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
    imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

    float x1 = cosImuYawStart * imuShiftFromStartXCur - sinImuYawStart * imuShiftFromStartZCur;
    float y1 = imuShiftFromStartYCur;
    float z1 = sinImuYawStart * imuShiftFromStartXCur + cosImuYawStart * imuShiftFromStartZCur;

    float x2 = x1;
    float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
    float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

    imuShiftFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
    imuShiftFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
    imuShiftFromStartZCur = z2;
}

void FeatureAssociation::VeloToStartIMU()
{
    imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
    imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
    imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

    float x1 = cosImuYawStart * imuVeloFromStartXCur - sinImuYawStart * imuVeloFromStartZCur;
    float y1 = imuVeloFromStartYCur;
    float z1 = sinImuYawStart * imuVeloFromStartXCur + cosImuYawStart * imuVeloFromStartZCur;

    float x2 = x1;
    float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
    float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

    imuVeloFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
    imuVeloFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
    imuVeloFromStartZCur = z2;
}

void FeatureAssociation::TransformToStartIMU(PointType *p)
{
    float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
    float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
    float z1 = p->z;

    float x2 = x1;
    float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
    float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

    float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
    float y3 = y2;
    float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

    float x4 = cosImuYawStart * x3 - sinImuYawStart * z3;
    float y4 = y3;
    float z4 = sinImuYawStart * x3 + cosImuYawStart * z3;

    float x5 = x4;
    float y5 = cosImuPitchStart * y4 + sinImuPitchStart * z4;
    float z5 = -sinImuPitchStart * y4 + cosImuPitchStart * z4;

    p->x = cosImuRollStart * x5 + sinImuRollStart * y5 + imuShiftFromStartXCur;
    p->y = -sinImuRollStart * x5 + cosImuRollStart * y5 + imuShiftFromStartYCur;
    p->z = z5 + imuShiftFromStartZCur;
}

void FeatureAssociation::adjustDistortion() {
  pcl::copyPointCloud(*segmentedCloud, *distortedCloud);
  bool halfPassed = false;
  int cloudSize = segmentedCloud->points.size();

  PointType point;

  for (int i = 0; i < cloudSize; i++) {
    point.x = segmentedCloud->points[i].y;
    point.y = segmentedCloud->points[i].z;
    point.z = segmentedCloud->points[i].x;

    float ori = -atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < segInfo.start_orientation - M_PI / 2)
        ori += 2 * M_PI;
      else if (ori > segInfo.start_orientation + M_PI * 3 / 2)
        ori -= 2 * M_PI;

      if (ori - segInfo.start_orientation > M_PI) halfPassed = true;
    } else {
      ori += 2 * M_PI;

      if (ori < segInfo.end_orientation - M_PI * 3 / 2)
        ori += 2 * M_PI;
      else if (ori > segInfo.end_orientation + M_PI / 2)
        ori -= 2 * M_PI;
    }

    float relTime = (ori - segInfo.start_orientation) / segInfo.orientation_diff;
    point.intensity =
        int(segmentedCloud->points[i].intensity) + _scan_period * relTime;
    
    distortedCloud->points[i] = point;
  //TODO
    if (imuPointerLast >= 0) 
      {
          float pointTime = relTime * _scan_period;
          imuPointerFront = imuPointerLastIteration;
          while (imuPointerFront != imuPointerLast) {
              if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
                  break;
              }
              imuPointerFront = (imuPointerFront + 1) % imuQueLength;
          }

          if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
              imuRollCur = imuRoll[imuPointerFront];
              imuPitchCur = imuPitch[imuPointerFront];
              imuYawCur = imuYaw[imuPointerFront];

              imuVeloXCur = imuVeloX[imuPointerFront];
              imuVeloYCur = imuVeloY[imuPointerFront];
              imuVeloZCur = imuVeloZ[imuPointerFront];

              imuShiftXCur = imuShiftX[imuPointerFront];
              imuShiftYCur = imuShiftY[imuPointerFront];
              imuShiftZCur = imuShiftZ[imuPointerFront];   
          } else {
              int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
              float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                                                / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
              float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                                              / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

              imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
              imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
              if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
                  imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
              } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
                  imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
              } else {
                  imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
              }

              imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
              imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
              imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

              imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
              imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
              imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
          }

          if (i == 0) {
              imuRollStart = imuRollCur;
              imuPitchStart = imuPitchCur;
              imuYawStart = imuYawCur;

              imuVeloXStart = imuVeloXCur;
              imuVeloYStart = imuVeloYCur;
              imuVeloZStart = imuVeloZCur;

              imuShiftXStart = imuShiftXCur;
              imuShiftYStart = imuShiftYCur;
              imuShiftZStart = imuShiftZCur;

              if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
                  imuAngularRotationXCur = imuAngularRotationX[imuPointerFront];
                  imuAngularRotationYCur = imuAngularRotationY[imuPointerFront];
                  imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront];
              }else{
                  int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                  float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                                                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                  float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                                                  / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                  imuAngularRotationXCur = imuAngularRotationX[imuPointerFront] * ratioFront + imuAngularRotationX[imuPointerBack] * ratioBack;
                  imuAngularRotationYCur = imuAngularRotationY[imuPointerFront] * ratioFront + imuAngularRotationY[imuPointerBack] * ratioBack;
                  imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront] * ratioFront + imuAngularRotationZ[imuPointerBack] * ratioBack;
              }

              imuAngularFromStartX = imuAngularRotationXCur - imuAngularRotationXLast;
              imuAngularFromStartY = imuAngularRotationYCur - imuAngularRotationYLast;
              imuAngularFromStartZ = imuAngularRotationZCur - imuAngularRotationZLast;

              imuAngularRotationXLast = imuAngularRotationXCur;
              imuAngularRotationYLast = imuAngularRotationYCur;
              imuAngularRotationZLast = imuAngularRotationZCur;

              updateImuRollPitchYawStartSinCos();
          } else {
              VeloToStartIMU();
              TransformToStartIMU(&point);
          }
      }
  // //TODO
  //   if (use_imu_undistortion){
  //     if (imuPointerLast >= 0) 
  //     {
  //         float pointTime = relTime * _scan_period;
  //         imuPointerFront = imuPointerLastIteration;
  //         while (imuPointerFront != imuPointerLast) {
  //             if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
  //                 break;
  //             }
  //             imuPointerFront = (imuPointerFront + 1) % imuQueLength;
  //         }

  //         if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
  //             imuRollCur = imuRoll[imuPointerFront];
  //             imuPitchCur = imuPitch[imuPointerFront];
  //             imuYawCur = imuYaw[imuPointerFront];

  //             imuVeloXCur = imuVeloX[imuPointerFront];
  //             imuVeloYCur = imuVeloY[imuPointerFront];
  //             imuVeloZCur = imuVeloZ[imuPointerFront];

  //             imuShiftXCur = imuShiftX[imuPointerFront];
  //             imuShiftYCur = imuShiftY[imuPointerFront];
  //             imuShiftZCur = imuShiftZ[imuPointerFront];   
  //         } else {
  //             int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
  //             float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
  //                                               / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
  //             float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
  //                                             / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

  //             imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
  //             imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
  //             if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
  //                 imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
  //             } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
  //                 imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
  //             } else {
  //                 imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
  //             }

  //             imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
  //             imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
  //             imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

  //             imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
  //             imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
  //             imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
  //         }

  //         if (i == 0) {
  //             imuRollStart = imuRollCur;
  //             imuPitchStart = imuPitchCur;
  //             imuYawStart = imuYawCur;

  //             imuVeloXStart = imuVeloXCur;
  //             imuVeloYStart = imuVeloYCur;
  //             imuVeloZStart = imuVeloZCur;

  //             imuShiftXStart = imuShiftXCur;
  //             imuShiftYStart = imuShiftYCur;
  //             imuShiftZStart = imuShiftZCur;

  //             if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
  //                 imuAngularRotationXCur = imuAngularRotationX[imuPointerFront];
  //                 imuAngularRotationYCur = imuAngularRotationY[imuPointerFront];
  //                 imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront];
  //             }else{
  //                 int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
  //                 float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
  //                                                   / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
  //                 float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
  //                                                 / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
  //                 imuAngularRotationXCur = imuAngularRotationX[imuPointerFront] * ratioFront + imuAngularRotationX[imuPointerBack] * ratioBack;
  //                 imuAngularRotationYCur = imuAngularRotationY[imuPointerFront] * ratioFront + imuAngularRotationY[imuPointerBack] * ratioBack;
  //                 imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront] * ratioFront + imuAngularRotationZ[imuPointerBack] * ratioBack;
  //             }

  //             imuAngularFromStartX = imuAngularRotationXCur - imuAngularRotationXLast;
  //             imuAngularFromStartY = imuAngularRotationYCur - imuAngularRotationYLast;
  //             imuAngularFromStartZ = imuAngularRotationZCur - imuAngularRotationZLast;

  //             imuAngularRotationXLast = imuAngularRotationXCur;
  //             imuAngularRotationYLast = imuAngularRotationYCur;
  //             imuAngularRotationZLast = imuAngularRotationZCur;

  //             updateImuRollPitchYawStartSinCos();
  //         } else {
  //             VeloToStartIMU();
  //             TransformToStartIMU(&point);
  //         }
  //     }
  //   }
    segmentedCloud->points[i] = point;
    // segmentedCloud->points[i].intensity = visualCloud->points[i].intensity;
  }
  imuPointerLastIteration = imuPointerLast;
}

void FeatureAssociation::calculateSmoothness() {
  int cloudSize = segmentedCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange = segInfo.segmented_cloud_range[i - 5] +
                      segInfo.segmented_cloud_range[i - 4] +
                      segInfo.segmented_cloud_range[i - 3] +
                      segInfo.segmented_cloud_range[i - 2] +
                      segInfo.segmented_cloud_range[i - 1] -
                      segInfo.segmented_cloud_range[i] * 10 +
                      segInfo.segmented_cloud_range[i + 1] +
                      segInfo.segmented_cloud_range[i + 2] +
                      segInfo.segmented_cloud_range[i + 3] +
                      segInfo.segmented_cloud_range[i + 4] +
                      segInfo.segmented_cloud_range[i + 5];

    cloudCurvature[i] = diffRange * diffRange;

    cloudNeighborPicked[i] = 0;
    cloudNeighborPickedPlane[i] = 0;
    cloudLabel[i] = 0;

    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void FeatureAssociation::calculateSmoothnessOurs() {
  int cloudSize = segmentedCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRangeX = 0;
    float diffRangeY = 0;
    float diffRangeZ = 0;
    for (int j = -5; j < 6; j++) {
      diffRangeX += segmentedCloud->points[i+j].x;
    }
    diffRangeX -= 11*segmentedCloud->points[i].x;
    for (int j = -5; j < 6; j++) {
      diffRangeY += segmentedCloud->points[i+j].y;
    }
    diffRangeY -= 11*segmentedCloud->points[i].y;
    for (int j = -5; j < 6; j++) {
      diffRangeZ += segmentedCloud->points[i+j].z;
    }
    diffRangeZ -= 11*segmentedCloud->points[i].z;
    float diffRange = sqrt(diffRangeX * diffRangeX + diffRangeY * diffRangeY + diffRangeZ * diffRangeZ)
                     /sqrt(segmentedCloud->points[i].x * segmentedCloud->points[i].x + segmentedCloud->points[i].y * segmentedCloud->points[i].y + segmentedCloud->points[i].z * segmentedCloud->points[i].z)/10;


    cloudCurvature[i] = diffRange;

    cloudNeighborPicked[i] = 0;
    cloudNeighborPickedPlane[i] = 0;
    cloudLabel[i] = 0;

    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}
// Eigen::Vector3f relocal_tt = relocal_pose.block<3,1>(0,3);

void FeatureAssociation::markOccludedPoints() {
  int cloudSize = segmentedCloud->points.size();

  for (int i = 5; i < cloudSize - 6; ++i) {
    float depth1 = segInfo.segmented_cloud_range[i];
    float depth2 = segInfo.segmented_cloud_range[i + 1];
    int columnDiff = std::abs(int(segInfo.segmented_cloud_col_ind[i + 1] -
                                  segInfo.segmented_cloud_col_ind[i]));

    if (columnDiff < 10) {
      if (depth1 - depth2 > 0.3) {
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
        cloudNeighborPickedPlane[i - 5] = 1;
        cloudNeighborPickedPlane[i - 4] = 1;
        cloudNeighborPickedPlane[i - 3] = 1;
        cloudNeighborPickedPlane[i - 2] = 1;
        cloudNeighborPickedPlane[i - 1] = 1;
        cloudNeighborPickedPlane[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
        cloudNeighborPickedPlane[i + 1] = 1;
        cloudNeighborPickedPlane[i + 2] = 1;
        cloudNeighborPickedPlane[i + 3] = 1;
        cloudNeighborPickedPlane[i + 4] = 1;
        cloudNeighborPickedPlane[i + 5] = 1;
        cloudNeighborPickedPlane[i + 6] = 1;
      }
    }

    float diff1 = std::abs(float(segInfo.segmented_cloud_range[i-1] - segInfo.segmented_cloud_range[i]));
    float diff2 = std::abs(float(segInfo.segmented_cloud_range[i+1] - segInfo.segmented_cloud_range[i]));

    if (diff1 > 0.02 * segInfo.segmented_cloud_range[i] &&
        diff2 > 0.02 * segInfo.segmented_cloud_range[i]){
      cloudNeighborPicked[i] = 1;
      cloudNeighborPickedPlane[i] = 1;
    }
  }
}

void FeatureAssociation::extractFeatures() {
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();

  for (int i = 0; i < _vertical_scans; i++) {
    surfPointsLessFlatScan->clear();

    for (int j = 0; j < 6; j++) {
      int sp =
          (segInfo.start_ring_index[i] * (6 - j) + segInfo.end_ring_index[i] * j) /
          6;
      int ep = (segInfo.start_ring_index[i] * (5 - j) +
                segInfo.end_ring_index[i] * (j + 1)) /
                   6 -
               1;

      if (sp >= ep) continue;

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep,
                by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > _edge_threshold &&
            segInfo.segmented_cloud_ground_flag[ind] == false) {
          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(segmentedCloud->points[ind]);
            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if ( ind + l >= static_cast<int>(segInfo.segmented_cloud_col_ind.size()) ) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
                             segInfo.segmented_cloud_col_ind[ind + l - 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if( ind + l < 0 ) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
                             segInfo.segmented_cloud_col_ind[ind + l + 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < _surf_threshold &&
            segInfo.segmented_cloud_ground_flag[ind] == true) {
          cloudLabel[ind] = -1;
          surfPointsFlat->push_back(segmentedCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if ( ind + l >= static_cast<int>(segInfo.segmented_cloud_col_ind.size()) ) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
                             segInfo.segmented_cloud_col_ind.at(ind + l - 1)));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if (ind + l < 0) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
                             segInfo.segmented_cloud_col_ind.at(ind + l + 1)));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
        }
      }
    }

    surfPointsLessFlatScanDS->clear();
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.filter(*surfPointsLessFlatScanDS);
    *surfPointsLessFlat += *surfPointsLessFlatScanDS;
  }
  // std::cout << "surfPointsFlat:" << surfPointsFlat->points.size() << std::endl;
  // std::cout << "cornerPointsLessSharp:" << cornerPointsLessSharp->points.size() << std::endl;
}

// void FeatureAssociation::extractFeaturesOurs() {
//   cornerPointsSharp->clear();
//   cornerPointsLessSharp->clear();
//   surfPointsFlat->clear();
//   surfPointsLessFlat->clear();

//   for (int i = 0; i < _vertical_scans; i++) {
//     surfPointsLessFlatScan->clear();
//     int sp = segInfo.start_ring_index[i];
//     int ep = segInfo.end_ring_index[i] - 1;
//     if (sp >= ep) continue;
//     std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

//     // Edge
//     for (int k = ep; k >= sp; k--) {
//       int ind = cloudSmoothness[k].ind;
//       if (cloudNeighborPicked[ind] == 0 &&
//           cloudCurvature[ind] > _edge_threshold &&
//           segInfo.segmented_cloud_ground_flag[ind] == false) {
//         cloudLabel[ind] = 1;
//         cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);


//         cloudNeighborPicked[ind] = 1;
//         for (int l = 1; l <= 5; l++) {
//           if ( ind + l >= static_cast<int>(segInfo.segmented_cloud_col_ind.size()) ) {
//             continue;
//           }
//           int columnDiff =
//               std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
//                             segInfo.segmented_cloud_col_ind[ind + l - 1]));
//           if (columnDiff > 10) break;
//           cloudNeighborPicked[ind + l] = 1;
//         }
//         for (int l = -1; l >= -5; l--) {
//           if( ind + l < 0 ) {
//             continue;
//           }
//           int columnDiff =
//               std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
//                             segInfo.segmented_cloud_col_ind[ind + l + 1]));
//           if (columnDiff > 10) break;
//           cloudNeighborPicked[ind + l] = 1;
//         }
//       }
//     }

//     // Planar
//     int smallestPickedNum = 0;
//     for (int k = sp; k <= ep; k++) {
//       int ind = cloudSmoothness[k].ind;
//       if (cloudNeighborPicked[ind] == 0 &&
//           cloudCurvature[ind] < _surf_threshold &&
//           segInfo.segmented_cloud_ground_flag[ind] == true) {
//         cloudLabel[ind] = -1;
//         surfPointsFlat->push_back(segmentedCloud->points[ind]);

//         smallestPickedNum++;
//         // if (smallestPickedNum >= 4) {
//         //   break;
//         // }

//         cloudNeighborPicked[ind] = 1;
//         for (int l = 1; l <= 5; l++) {
//           if ( ind + l >= static_cast<int>(segInfo.segmented_cloud_col_ind.size()) ) {
//             continue;
//           }
//           int columnDiff =
//               std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
//                             segInfo.segmented_cloud_col_ind.at(ind + l - 1)));
//           if (columnDiff > 10) break;

//           cloudNeighborPicked[ind + l] = 1;
//         }
//         for (int l = -1; l >= -5; l--) {
//           if (ind + l < 0) {
//             continue;
//           }
//           int columnDiff =
//               std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
//                             segInfo.segmented_cloud_col_ind.at(ind + l + 1)));
//           if (columnDiff > 10) break;

//           cloudNeighborPicked[ind + l] = 1;
//         }
//       }
//     }

//     for (int k = sp; k <= ep; k++) {
//       if (cloudLabel[k] <= 0) {
//         surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
//       }
//     }
//     surfPointsLessFlatScanDS->clear();
//     downSizeFilter.setInputCloud(surfPointsLessFlatScan);
//     downSizeFilter.filter(*surfPointsLessFlatScanDS);
//     *surfPointsLessFlat += *surfPointsLessFlatScanDS;
//   }
//   // *cornerPointsSharp = *cornerPointsLessSharp;
//   // DBSCAN Refined Edge Feature
//   DBSCAN_EdgeFeature();
//   // DEBUG
//   // for (int i = 0; i < (int)cluster.size(); ++i) {
//   //       std::cout << cluster[i] << "; ";
//   // }
//   // std::cout << std::endl;
//   cluster_length = cluster;
//   std::sort(cluster_length.begin(), cluster_length.end());
//   cluster_num_list.clear();
//   int cluster_cnt = 1;
//   for (int i = 0; i < (int)cluster_length.size()-1; i++) {
//     if (cluster_length[i+1] - cluster_length[i] == 0){
//       cluster_cnt++;
//     }else{
//       cluster_num_list.push_back(cluster_cnt);
//       cluster_cnt = 1;
//     }
//   }
//   cluster_inlier.clear();
//   for (int i = 0; i < (int)cluster_num_list.size(); i++) {
//     if (cluster_num_list[i]>=4){
//       cluster_inlier.push_back(i+1);
//     }
//   }
//   for (int i = 0; i < (int)cluster.size(); i++) {
//     for (int j = 0; j < (int)cluster_inlier.size(); j++){
//       if (cluster[i] == cluster_inlier[j]){
//         cornerPointsSharp->push_back(cornerPointsLessSharp->points[i]);
//       }
//     }
//   }
//   // std::cout << "surfPointsFlat:" << surfPointsFlat->points.size() << std::endl;
//   // std::cout << "cornerPointsSharp:" << cornerPointsSharp->points.size() << std::endl;
// }

void FeatureAssociation::extractFeaturesOurs() {
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();

  for (int i = 0; i < _vertical_scans; i++) { // _vertical_scans = 16,對每一條掃描線取特徵
    surfPointsLessFlatScan->clear();
    int sp = segInfo.start_ring_index[i];
    int ep = segInfo.end_ring_index[i] - 1;
    // std::cout << "sp = " << sp << std::endl;
    // std::cout << "ep = " << ep << std::endl;
    if (sp >= ep) continue;
    std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

    // Edge
    for (int k = ep; k >= sp; k--) {
      int ind = cloudSmoothness[k].ind;
      if (cloudNeighborPicked[ind] == 0 &&
          cloudCurvature[ind] > _edge_threshold &&
          segInfo.segmented_cloud_ground_flag[ind] == false) {
        cloudLabel[ind] = 1;
        cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
        // std::cout << "cornerPointsLessSharp.size() = " << cornerPointsLessSharp -> points.size() << std::endl;


        cloudNeighborPicked[ind] = 1;
        for (int l = 1; l <= 5; l++) {
          if ( ind + l >= static_cast<int>(segInfo.segmented_cloud_col_ind.size()) ) {
            continue;
          }
          int columnDiff =
              std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
                            segInfo.segmented_cloud_col_ind[ind + l - 1]));
          if (columnDiff > 10) break;
          cloudNeighborPicked[ind + l] = 1;
        }
        for (int l = -1; l >= -5; l--) {
          if( ind + l < 0 ) {
            continue;
          }
          int columnDiff =
              std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
                            segInfo.segmented_cloud_col_ind[ind + l + 1]));
          if (columnDiff > 10) break;
          cloudNeighborPicked[ind + l] = 1;
        }
      }
    }

    // Planar
    int smallestPickedNum = 0; // 初始化一個變量 smallestPickedNum 用於記錄選取的平面點的數量。
    for (int k = sp; k <= ep; k++) { // 通過一個循環，從 sp 開始到 ep 結束，遍歷某個範圍內的點雲數據。這個範圍可能是一個掃描線或是點雲數據的一個子集。
      int ind = cloudSmoothness[k].ind; // 對於每個點 k，獲取它的索引 ind，這個索引是點在點雲數據中的位置。
      // 檢查點 ind 是否已經被選取過（cloudNeighborPicked[ind] == 0），
      // 其曲率是否小於一個閾值（cloudCurvature[ind] < _surf_threshold），
      // 以及該點是否被標記為地面點（segInfo.segmented_cloud_ground_flag[ind] == true）。
      // 這些條件用於篩選出可能屬於平面的點。
      if (cloudNeighborPicked[ind] == 0 &&  
          cloudCurvature[ind] < _surf_threshold &&
          segInfo.segmented_cloud_ground_flag[ind] == true) {
        // 如果一個點滿足以上條件，它將被標記為 -1，表示它被認為是一個平面點，並將其添加到 surfPointsFlat 容器中，用於存儲平面點。 
        // surfPointsFlat會在findCorrespondingSurfFeatures()中被調用 會在TransformToStart()先轉換到起始座標系 接著放到kdtree中找特徵匹配點
        cloudLabel[ind] = -1; 
        surfPointsFlat->push_back(segmentedCloud->points[ind]);
        // std::cout << "surfPointsFlat.size() = " << surfPointsFlat -> points.size() << std::endl;

        smallestPickedNum++;  // 更新 smallestPickedNum，記錄已選取的平面點數量。
        // if (smallestPickedNum >= 4) {
        //   break;
        // } // 原本平面點每個部份只取4個點

        cloudNeighborPicked[ind] = 1; // 將當前點標記為已被選取（cloudNeighborPicked[ind] = 1），以避免它被重覆選取。
        // 對當前點的相鄰點進行遍歷，檢查它們是否在同一列（在一定範圍內），
        // 並根據它們的列索引差異決定是否將它們也標記為已選取。
        // 這樣做是為了在提取平面特征點時考慮點雲的連續性，避免將非平面區域的點錯誤地選為平面點。
        for (int l = 1; l <= 5; l++) {
          if ( ind + l >= static_cast<int>(segInfo.segmented_cloud_col_ind.size()) ) {
            continue;
          }
          int columnDiff =
              std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
                            segInfo.segmented_cloud_col_ind.at(ind + l - 1)));
          if (columnDiff > 10) break;

          cloudNeighborPicked[ind + l] = 1;
        }
        // 類似地，對當前點之前的相鄰點進行檢查，應用同樣的邏輯。
        for (int l = -1; l >= -5; l--) {
          if (ind + l < 0) {
            continue;
          }
          int columnDiff =
              std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
                            segInfo.segmented_cloud_col_ind.at(ind + l + 1)));
          if (columnDiff > 10) break;

          cloudNeighborPicked[ind + l] = 1;
        }
      }
    }
    
    // 另一個循環用於遍歷相同的點雲範圍，將所有未被標記為平面點的點（cloudLabel[k] <= 0）添加到 surfPointsLessFlatScan 容器中。這可能包括一些不那麽平坦的點，但依然可能對後續處理有用。
    for (int k = sp; k <= ep; k++) {
      if (cloudLabel[k] <= 0) {
        surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
      }
    }
    // 使用一個降采樣濾波器（downSizeFilter）對 surfPointsLessFlatScan 中的點進行降采樣，減少數據量以提高處理效率。處理後的點雲存儲在 surfPointsLessFlatScanDS 容器中。
    surfPointsLessFlatScanDS->clear();
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.filter(*surfPointsLessFlatScanDS);
    *surfPointsLessFlat += *surfPointsLessFlatScanDS;
  }
  // *cornerPointsSharp = *cornerPointsLessSharp;
  // DBSCAN Refined Edge Feature
  DBSCAN_EdgeFeature();
  // DEBUG
  // for (int i = 0; i < (int)cluster.size(); ++i) {
  //       std::cout << cluster[i] << "; ";
  // }
  // std::cout << std::endl;
  cluster_length = cluster;
  std::sort(cluster_length.begin(), cluster_length.end());
  cluster_num_list.clear();
  int cluster_cnt = 1;
  for (int i = 0; i < (int)cluster_length.size()-1; i++) {
    if (cluster_length[i+1] - cluster_length[i] == 0){
      cluster_cnt++;
    }else{
      cluster_num_list.push_back(cluster_cnt);
      cluster_cnt = 1;
    }
  }
  cluster_inlier.clear();
  for (int i = 0; i < (int)cluster_num_list.size(); i++) {
    if (cluster_num_list[i]>=4){
      cluster_inlier.push_back(i+1);
    }
  }
  for (int i = 0; i < (int)cluster.size(); i++) {
    for (int j = 0; j < (int)cluster_inlier.size(); j++){
      if (cluster[i] == cluster_inlier[j]){
        cornerPointsSharp->push_back(cornerPointsLessSharp->points[i]);
      }
    }
  }
  // std::cout << "surfPointsFlat:" << surfPointsFlat->points.size() << std::endl;
  // std::cout << "cornerPointsSharp:" << cornerPointsSharp->points.size() << std::endl;

  // Alex Shadow point push back
  for (const auto& point : *virtualshadowCloud) {
      surfPointsFlat->push_back(point);
      // std::cout << "virtualshadowCloud: x=" << point.x << ", y=" << point.y << ", z=" << point.z << ", intensity=" << point.intensity << std::endl;
      // std::cout << "shadow iter : " << shadow_i << std::endl;
  }
  // surfPointsFlat->push_back(virtualshadowCloud->virtualshadowpoint);
}

void FeatureAssociation::DBSCAN_EdgeFeature(){
  // Input: cornerPointsLessSharp
  // Output: cornerPointsSharp
  int label = 0;
  cluster.clear();
  cluster.resize(cornerPointsLessSharp->points.size());
  kxy.clear();
  kz.clear();
  for (int i = 0; i < (int)cornerPointsLessSharp->points.size(); i++) {
    float x0 = cornerPointsLessSharp->points[i].z; // coordinate from loam to lidar
    float y0 = cornerPointsLessSharp->points[i].x;
    float z0 = cornerPointsLessSharp->points[i].y;
    float AB = std::atan2(z0,sqrt(x0 * x0 + y0 * y0));
    float kxy0 = sqrt(x0 * x0 + y0 * y0) * std::sin(_ang_resolution_X) * RatioXY;
    float kz0 = (sqrt(x0 * x0 + y0 * y0) * std::tan(AB + _ang_resolution_Y) - sqrt(x0 * x0 + y0 * y0) * std::tan(AB - _ang_resolution_Y))/2*RatioZ;
    // std::cout << std::sin(_ang_resolution_X) * RatioXY << "; ";
    kxy.push_back(kxy0);
    kz.push_back(kz0);
  }
  // std::cout << std::endl;
  // for (int i = 0; i < (int)kz.size(); ++i) {
  //   std::cout << kz[i] << "; ";
  // }
  // std::cout << std::endl;
  for (int i = 0; i < (int)cornerPointsLessSharp->points.size(); i++) {
    cluster[i] = 0;
    float x0 = cornerPointsLessSharp->points[i].z;
    float y0 = cornerPointsLessSharp->points[i].x;
    float z0 = cornerPointsLessSharp->points[i].y;
    in_idx.clear();
    in_label_list.clear();
    for (int j = 0; j < (int)cornerPointsLessSharp->points.size(); j++) {
      float xj = cornerPointsLessSharp->points[j].z;
      float yj = cornerPointsLessSharp->points[j].x;
      float zj = cornerPointsLessSharp->points[j].y;
      float eps = sqrt((x0-xj)*(x0-xj)/(kxy[j]*kxy[j]) + (y0-yj)*(y0-yj)/(kxy[j]*kxy[j]) + (z0-zj)*(z0-zj)/(kz[j]*kz[j]));
      if (eps <= DBFr){
        in_idx.push_back(j);
        in_label_list.push_back(cluster[j]);
      }
    }
    int min_label = 999999999;
    for (int j = 0; j < (int)in_idx.size(); j++) {
      if (cluster[in_idx[j]] != 0 && cluster[in_idx[j]] < min_label){
        min_label = cluster[in_idx[j]];
      }
    }
    if (min_label<=label){
      std::sort(in_label_list.begin(), in_label_list.end());
      auto last = std::unique(in_label_list.begin(), in_label_list.end());
      in_label_list.erase(last, in_label_list.end());
      for (int j = 0; j < (int)cluster.size(); j++){
        for (int k = 0; k < (int)in_label_list.size(); k++){
          if (cluster[j] == in_label_list[k]){
            cluster[j] = min_label;
          }
        }
      }
      for (int j = 0; j < (int)in_idx.size(); j++) {
        cluster[in_idx[j]] = min_label;
      }
      
    }else{
      label +=1;
      for (int j = 0; j < (int)in_idx.size(); j++){
        cluster[in_idx[j]] = label;
      }
    }
  }
}

void FeatureAssociation::TransformToStart(PointType const * const pi, PointType * const po)
{
    float s = 10 * (pi->intensity - int(pi->intensity));

    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    po->x = cos(ry) * x2 - sin(ry) * z2;
    po->y = y2;
    po->z = sin(ry) * x2 + cos(ry) * z2;
    po->intensity = pi->intensity;
}

void FeatureAssociation::TransformToEnd(PointType const * const pi, PointType * const po)
{
    float s = 10 * (pi->intensity - int(pi->intensity));

    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    float x3 = cos(ry) * x2 - sin(ry) * z2;
    float y3 = y2;
    float z3 = sin(ry) * x2 + cos(ry) * z2;

    rx = transformCur[0];
    ry = transformCur[1];
    rz = transformCur[2];
    tx = transformCur[3];
    ty = transformCur[4];
    tz = transformCur[5];

    float x4 = cos(ry) * x3 + sin(ry) * z3;
    float y4 = y3;
    float z4 = -sin(ry) * x3 + cos(ry) * z3;

    float x5 = x4;
    float y5 = cos(rx) * y4 - sin(rx) * z4;
    float z5 = sin(rx) * y4 + cos(rx) * z4;

    float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
    float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
    float z6 = z5 + tz;

    if(use_imu_undistortion){

      float x7 = cosImuRollStart * (x6 - imuShiftFromStartX) 
                - sinImuRollStart * (y6 - imuShiftFromStartY);
      float y7 = sinImuRollStart * (x6 - imuShiftFromStartX) 
                + cosImuRollStart * (y6 - imuShiftFromStartY);
      float z7 = z6 - imuShiftFromStartZ;

      float x8 = x7;
      float y8 = cosImuPitchStart * y7 - sinImuPitchStart * z7;
      float z8 = sinImuPitchStart * y7 + cosImuPitchStart * z7;

      float x9 = cosImuYawStart * x8 + sinImuYawStart * z8;
      float y9 = y8;
      float z9 = -sinImuYawStart * x8 + cosImuYawStart * z8;

      float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
      float y10 = y9;
      float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

      float x11 = x10;
      float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
      float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

      po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
      po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
      po->z = z11;
      po->intensity = int(pi->intensity);
    }else{
      po->x = x6;
      po->y = y6;
      po->z = z6;
      po->intensity = int(pi->intensity);
    }
}

void FeatureAssociation::PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
                           float alx, float aly, float alz, float &acx, float &acy, float &acz)
{
    float sbcx = sin(bcx);
    float cbcx = cos(bcx);
    float sbcy = sin(bcy);
    float cbcy = cos(bcy);
    float sbcz = sin(bcz);
    float cbcz = cos(bcz);

    float sblx = sin(blx);
    float cblx = cos(blx);
    float sbly = sin(bly);
    float cbly = cos(bly);
    float sblz = sin(blz);
    float cblz = cos(blz);

    float salx = sin(alx);
    float calx = cos(alx);
    float saly = sin(aly);
    float caly = cos(aly);
    float salz = sin(alz);
    float calz = cos(alz);

    float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
              - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
              - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
              - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
              - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
    acx = -asin(srx);

    float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                  - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                  - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
                  + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
    float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
                  - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                  - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                  + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
    acy = atan2(srycrx / cos(acx), crycrx / cos(acx));
    
    float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
                  - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
                  - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
                  + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
                  - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
                  + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
                  + calx*cblx*salz*sblz);
    float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
                  - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
                  + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
                  + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
                  + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
                  - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
                  - calx*calz*cblx*sblz);
    acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
}

void FeatureAssociation::AccumulateRotation(float cx, float cy, float cz,
                                            float lx, float ly, float lz,
                                            float &ox, float &oy, float &oz) {
  float srx = cos(lx) * cos(cx) * sin(ly) * sin(cz) -
              cos(cx) * cos(cz) * sin(lx) - cos(lx) * cos(ly) * sin(cx);
  ox = -asin(srx);

  float srycrx =
      sin(lx) * (cos(cy) * sin(cz) - cos(cz) * sin(cx) * sin(cy)) +
      cos(lx) * sin(ly) * (cos(cy) * cos(cz) + sin(cx) * sin(cy) * sin(cz)) +
      cos(lx) * cos(ly) * cos(cx) * sin(cy);
  float crycrx =
      cos(lx) * cos(ly) * cos(cx) * cos(cy) -
      cos(lx) * sin(ly) * (cos(cz) * sin(cy) - cos(cy) * sin(cx) * sin(cz)) -
      sin(lx) * (sin(cy) * sin(cz) + cos(cy) * cos(cz) * sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx =
      sin(cx) * (cos(lz) * sin(ly) - cos(ly) * sin(lx) * sin(lz)) +
      cos(cx) * sin(cz) * (cos(ly) * cos(lz) + sin(lx) * sin(ly) * sin(lz)) +
      cos(lx) * cos(cx) * cos(cz) * sin(lz);
  float crzcrx =
      cos(lx) * cos(lz) * cos(cx) * cos(cz) -
      cos(cx) * sin(cz) * (cos(ly) * sin(lz) - cos(lz) * sin(lx) * sin(ly)) -
      sin(cx) * (sin(ly) * sin(lz) + cos(ly) * cos(lz) * sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void FeatureAssociation::findCorrespondingCornerFeatures(int iterCount) {
  int cornerPointsSharpNum = cornerPointsSharp->points.size();

  for (int i = 0; i < cornerPointsSharpNum; i++) {
    PointType pointSel;
    // pointSel = cornerPointsSharp->points[i];
    TransformToStart(&cornerPointsSharp->points[i], &pointSel);

    if (iterCount % 5 == 0) {
      kdtreeCornerLast.nearestKSearch(pointSel, 1, pointSearchInd,
                                       pointSearchSqDis);
      int closestPointInd = -1, minPointInd2 = -1;

      if (pointSearchSqDis[0] < _nearest_feature_dist_sqr) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan =
            int(laserCloudCornerLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = _nearest_feature_dist_sqr;
        for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
          if (int(laserCloudCornerLast->points[j].intensity) >
              closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                           (laserCloudCornerLast->points[j].x - pointSel.x) +
                       (laserCloudCornerLast->points[j].y - pointSel.y) *
                           (laserCloudCornerLast->points[j].y - pointSel.y) +
                       (laserCloudCornerLast->points[j].z - pointSel.z) *
                           (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) >
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudCornerLast->points[j].intensity) <
              closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                           (laserCloudCornerLast->points[j].x - pointSel.x) +
                       (laserCloudCornerLast->points[j].y - pointSel.y) *
                           (laserCloudCornerLast->points[j].y - pointSel.y) +
                       (laserCloudCornerLast->points[j].z - pointSel.z) *
                           (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) <
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
      }

      pointSearchCornerInd1[i] = closestPointInd;
      pointSearchCornerInd2[i] = minPointInd2;
    }

    if (pointSearchCornerInd2[i] >= 0) {
      PointType tripod1 =
          laserCloudCornerLast->points[pointSearchCornerInd1[i]];
      PointType tripod2 =
          laserCloudCornerLast->points[pointSearchCornerInd2[i]];

      float x0 = pointSel.x;
      float y0 = pointSel.y;
      float z0 = pointSel.z;
      float x1 = tripod1.x;
      float y1 = tripod1.y;
      float z1 = tripod1.z;
      float x2 = tripod2.x;
      float y2 = tripod2.y;
      float z2 = tripod2.z;

      float m11 = ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1));
      float m22 = ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1));
      float m33 = ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1));

      float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

      float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                       (z1 - z2) * (z1 - z2));

      float la = ((y1 - y2) * m11 + (z1 - z2) * m22) / a012 / l12;

      float lb = -((x1 - x2) * m11 - (z1 - z2) * m33) / a012 / l12;

      float lc = -((x1 - x2) * m22 + (y1 - y2) * m33) / a012 / l12;

      float ld2 = a012 / l12;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 - 1.8 * fabs(ld2);
      }

      if (s > 0.1 && ld2 != 0) {
        PointType coeff;
        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        laserCloudOri->push_back(cornerPointsSharp->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

void FeatureAssociation::findCorrespondingSurfFeatures(int iterCount) {
  int surfPointsFlatNum = surfPointsFlat->points.size();

  for (int i = 0; i < surfPointsFlatNum; i++) {
    // std::cout << "knn times : " << i << std::endl;
    PointType pointSel;
    // pointSel = surfPointsFlat->points[i];
    TransformToStart(&surfPointsFlat->points[i], &pointSel);
    std::cout << "pointSel's idx : " << pointSel.intensity << std::endl;
    
    // ----------------------------------------------------------------------------------------
    // 每5次迭代找一次最近鄰點
    //************************************
    // Method:    nearestKSearch k-近邻搜索，搜索离point最近的k个点
    // 注意此方法不对输入索引进行任何检查（即index >= cloud.points.size（） || index < 0），并假定是有效（即有限）数据。
    // FullName:  pcl::KdTreeFLANN<PointT, Dist>::nearestKSearch
    // Access:    public 
    // Returns:   int 返回搜索到的点的个数
    // Parameter: const PointT & point 搜索离point最近的k个点
    // Parameter: int k 搜索离point最近的k个点
    // Parameter: std::vector<int> & k_indices 搜索到的点在数据源中的下标
    // Parameter: std::vector<float> & k_distances point到被搜索点的距离，与下标相对应
    //************************************
    if (iterCount % 5 == 0) {
      kdtreeSurfLast.nearestKSearch(pointSel, 1, pointSearchInd,
                                     pointSearchSqDis);
      laserCloudSurfLast;
      std::cout << "correspondence's idx : " << laserCloudSurfLast->points[pointSearchInd[0]].intensity << std::endl;
      int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
      // std::cout << "Vector elements are (using range-based for loop): ";
      // for (const auto& value : pointSearchInd) {
      //     std::cout << value << " ";
      // }
      // std::cout << std::endl;

      if (pointSearchSqDis[0] < _nearest_feature_dist_sqr) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan =
            int(laserCloudSurfLast->points[closestPointInd].intensity);
            // std::cout << "closestPointScan = " << closestPointScan << std::endl;

        float pointSqDis, minPointSqDis2 = _nearest_feature_dist_sqr,
                          minPointSqDis3 = _nearest_feature_dist_sqr;
        for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
          if (int(laserCloudSurfLast->points[j].intensity) >
              closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                           (laserCloudSurfLast->points[j].x - pointSel.x) +
                       (laserCloudSurfLast->points[j].y - pointSel.y) *
                           (laserCloudSurfLast->points[j].y - pointSel.y) +
                       (laserCloudSurfLast->points[j].z - pointSel.z) *
                           (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) <=
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudSurfLast->points[j].intensity) <
              closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                           (laserCloudSurfLast->points[j].x - pointSel.x) +
                       (laserCloudSurfLast->points[j].y - pointSel.y) *
                           (laserCloudSurfLast->points[j].y - pointSel.y) +
                       (laserCloudSurfLast->points[j].z - pointSel.z) *
                           (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) >=
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
      }

      pointSearchSurfInd1[i] = closestPointInd; // 最近鄰點
      pointSearchSurfInd2[i] = minPointInd2;
      pointSearchSurfInd3[i] = minPointInd3;
    }
    // ----------------------------------------------------------------------------------------
    // 點到平面距離
    if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
      PointType tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
      PointType tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
      PointType tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];
      // 平面方程係數
      float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) -
                 (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
      float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) -
                 (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
      float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) -
                 (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
      float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

      float ps = sqrt(pa * pa + pb * pb + pc * pc);

      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 -
            1.8 * fabs(pd2) /
                sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y +
                          pointSel.z * pointSel.z));
      }

      if (s > 0.1 && pd2 != 0) {
        PointType coeff;
        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        laserCloudOri->push_back(surfPointsFlat->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

bool FeatureAssociation::calculateTransformationSurf(int iterCount) {
  int pointSelNum = laserCloudOri->points.size(); // 获取原始点云中点的数量

  // 初始化矩阵和向量用于后续的线性方程求解
  Eigen::Matrix<float,Eigen::Dynamic,3> matA(pointSelNum, 3); // 系数矩阵A
  Eigen::Matrix<float,3,Eigen::Dynamic> matAt(3,pointSelNum); // A的转置
  Eigen::Matrix<float,3,3> matAtA; // A转置乘以A
  Eigen::VectorXf matB(pointSelNum); // 常数向量B
  Eigen::Matrix<float,3,1> matAtB; // A转置乘以B
  Eigen::Matrix<float,3,1> matX; // 解向量X
  Eigen::Matrix<float,3,3> matP; // 奇异性调整矩阵
  // std::cout << "Original"<< ": " << transformCur[0] << "," << transformCur[1] << "," << transformCur[2] << std::endl; // debug

  //TODO
  // Eigen::Matrix3d imuMatPre;
  // Eigen::Matrix3d imuMatCur;
  // imuMatPre = Eigen::AngleAxisd(imuRollPreSurf, Eigen::Vector3d::UnitZ())
  //                    * Eigen::AngleAxisd(imuPitchPreSurf, Eigen::Vector3d::UnitY())
  //                    * Eigen::AngleAxisd(imuRollPreSurf, Eigen::Vector3d::UnitX());
  // Eigen::Matrix3d imuMatPre_inv = imuMatPre.inverse();
  // imuMatCur = Eigen::AngleAxisd(imuRoll[imuPointerLast], Eigen::Vector3d::UnitZ())
  //                    * Eigen::AngleAxisd(imuPitch[imuPointerLast], Eigen::Vector3d::UnitY())
  //                    * Eigen::AngleAxisd(imuYaw[imuPointerLast], Eigen::Vector3d::UnitX());
  // Eigen::Matrix3d deltaImuMat = imuMatCur * imuMatPre_inv; 
  // Eigen::Vector3d deltaImuEuler = deltaImuMat.eulerAngles(2,1,0);
  // // transformCur[0] = - deltaImuEuler[1];
  // // transformCur[1] = - deltaImuEuler[2];
  // // transformCur[2] = - deltaImuEuler[0];
  // transformCur[0] = - imuAngularFromStartY;
  // transformCur[1] = - imuAngularFromStartZ;
  // transformCur[2] =  imuAngularFromStartX;
  // transformCur[3] += imuVeloFromStartYCur * _scan_period;
  // transformCur[4] += imuVeloFromStartZCur * _scan_period;
  // transformCur[5] -= imuVeloFromStartXCur * _scan_period;
  // // std::cout << "After"<< ": " << transformCur[0] << "," << transformCur[1] << "," << transformCur[2] << std::endl;  // debug
  // imuRollPreSurf = imuRoll[imuPointerLast];
  // imuPitchPreSurf = imuPitch[imuPointerLast];
  // imuRollPreSurf = imuYaw[imuPointerLast];
  //TODO
  // 计算当前变换的sin和cos值，简化后续计算
  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  // 获取当前的平移量
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  // 下面的 a1 至 a11, b1, b2, b5, b6, c1 至 c9 是根据旋转和平移计算得出的中间变量
  float a1 = crx * sry * srz;
  float a2 = crx * crz * sry;
  float a3 = srx * sry;
  float a4 = tx * a1 - ty * a2 - tz * a3;
  float a5 = srx * srz;
  float a6 = crz * srx;
  float a7 = ty * a6 - tz * crx - tx * a5;
  float a8 = crx * cry * srz;
  float a9 = crx * cry * crz;
  float a10 = cry * srx;
  float a11 = tz * a10 + ty * a9 - tx * a8;

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;

  float c1 = -b6;
  float c2 = b5;
  float c3 = tx * b6 - ty * b5;
  float c4 = -crx * crz;
  float c5 = crx * srz;
  float c6 = ty * c5 + tx * -c4;
  float c7 = b2;
  float c8 = -b1;
  float c9 = tx * -b2 - ty * -b1;

  // 构建系数矩阵和常数向量
  for (int i = 0; i < pointSelNum; i++) {
    PointType pointOri = laserCloudOri->points[i]; // 原始点
    PointType coeff = coeffSel->points[i]; // 对应的系数

    // 使用变换和系数计算线性方程组的系数
    // arx, arz, aty 是由点的原始位置、当前估计的变换和系数计算的线性方程的系数
    // 下面是系数的具体计算公式
    float arx =
        (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x +
        (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y +
        (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

    float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x +
                (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y +
                (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

    float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

    float d2 = coeff.intensity;  // 权重因子或其他形式的系数

    matA(i, 0) = arx;
    matA(i, 1) = arz;
    matA(i, 2) = aty;
    matB(i, 0) = -0.05 * d2; // 根据权重调整常数项
  }

  // 通过矩阵运算求解线性方程组
  matAt = matA.transpose(); // 计算A的转置
  matAtA = matAt * matA; // 计算AtA
  matAtB = matAt * matB; // 计算AtB
  matX = matAtA.colPivHouseholderQr().solve(matAtB); // 求解线性方程组得到X

  // 如果是第一次迭代，检查是否存在奇异性，并准备相应的调整
  if (iterCount == 0) {
    // Eigen库中的自伴特征求解器用于计算特征值和特征向量
    Eigen::Matrix<float,1,3> matE; // 特征值
    Eigen::Matrix<float,3,3> matV; // 特征向量
    Eigen::Matrix<float,3,3> matV2; // 用于奇异性调整的特征向量

    // 求解特征值和特征向量
    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,3,3> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
          matV2(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate) {
    Eigen::Matrix<float,3,1> matX2;
    matX2 = matX;
    matX = matP * matX2;
  }

  // 更新变换估计
  transformCur[0] += matX(0, 0);
  transformCur[2] += matX(1, 0);
  transformCur[4] += matX(2, 0);

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transformCur[i])) transformCur[i] = 0;
  }

  // 收敛检查
  float deltaR = sqrt(pow(RAD2DEG * (matX(0, 0)), 2) +
                      pow(RAD2DEG * (matX(1, 0)), 2));
  float deltaT = sqrt(pow(matX(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}


bool FeatureAssociation::calculateTransformationCorner(int iterCount) {
  int pointSelNum = laserCloudOri->points.size();

  Eigen::Matrix<float,Eigen::Dynamic,3> matA(pointSelNum, 3);
  Eigen::Matrix<float,3,Eigen::Dynamic> matAt(3,pointSelNum);
  Eigen::Matrix<float,3,3> matAtA;
  Eigen::VectorXf matB(pointSelNum);
  Eigen::Matrix<float,3,1> matAtB;
  Eigen::Matrix<float,3,1> matX;
  Eigen::Matrix<float,3,3> matP;
  // TODO
  // Eigen::Matrix3d imuMatPre;
  // Eigen::Matrix3d imuMatCur;
  // imuMatPre = Eigen::AngleAxisd(imuRollPreCorner, Eigen::Vector3d::UnitZ())
  //                    * Eigen::AngleAxisd(imuPitchPreCorner, Eigen::Vector3d::UnitY())
  //                    * Eigen::AngleAxisd(imuRollPreCorner, Eigen::Vector3d::UnitX());
  // Eigen::Matrix3d imuMatPre_inv = imuMatPre.inverse();
  // imuMatCur = Eigen::AngleAxisd(imuRoll[imuPointerLast], Eigen::Vector3d::UnitZ())
  //                    * Eigen::AngleAxisd(imuPitch[imuPointerLast], Eigen::Vector3d::UnitY())
  //                    * Eigen::AngleAxisd(imuYaw[imuPointerLast], Eigen::Vector3d::UnitX());
  // Eigen::Matrix3d deltaImuMat = imuMatCur * imuMatPre_inv; 
  // Eigen::Vector3d deltaImuEuler = deltaImuMat.eulerAngles(2,1,0);
  // // transformCur[0] = - deltaImuEuler[1];
  // // transformCur[1] = - deltaImuEuler[2];
  // // transformCur[2] = - deltaImuEuler[0];
  // transformCur[0] = - imuAngularFromStartY;
  // transformCur[1] = - imuAngularFromStartZ;
  // transformCur[2] =  imuAngularFromStartX;
  // transformCur[3] += imuVeloFromStartYCur * _scan_period;
  // transformCur[4] += imuVeloFromStartZCur * _scan_period;
  // transformCur[5] -= imuVeloFromStartXCur * _scan_period;
  // imuRollPreCorner = imuRoll[imuPointerLast];
  // imuPitchPreCorner = imuPitch[imuPointerLast];
  // imuRollPreCorner = imuYaw[imuPointerLast];
  
  // std::cout << "Precompensation"   << imuRoll[imuPointerLast] << "," << imuPitch[imuPointerLast] << "," << imuYaw[imuPointerLast] << std::endl; //Alex debug
  //TODO
  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b3 = crx * cry;
  float b4 = tx * -b1 + ty * -b2 + tz * b3;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;
  float b7 = crx * sry;
  float b8 = tz * b7 - ty * b6 - tx * b5;

  float c5 = crx * srz;

  for (int i = 0; i < pointSelNum; i++) {
    PointType pointOri = laserCloudOri->points[i];
    PointType coeff = coeffSel->points[i];

    float ary =
        (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x +
        (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

    float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

    float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

    float d2 = coeff.intensity;

    matA(i, 0) = ary;
    matA(i, 1) = atx;
    matA(i, 2) = atz;
    matB(i, 0) = -0.05 * d2;
  }

  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1, 3> matE;
    Eigen::Matrix<float,3, 3> matV;
    Eigen::Matrix<float,3, 3> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,3,3> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
          matV2(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate) {
    Eigen::Matrix<float,3,1> matX2;
    matX2 = matX;
    matX = matP * matX2;
  }

  transformCur[1] += matX(0, 0);
  transformCur[3] += matX(1, 0);
  transformCur[5] += matX(2, 0);

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transformCur[i])) transformCur[i] = 0;
  }

  float deltaR = sqrt(pow(RAD2DEG * (matX(0, 0)), 2));
  float deltaT = sqrt(pow(matX(1, 0) * 100, 2) +
                      pow(matX(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

bool FeatureAssociation::calculateTransformation(int iterCount) {
  int pointSelNum = laserCloudOri->points.size();

  Eigen::Matrix<float,Eigen::Dynamic,6> matA(pointSelNum, 6);
  Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,pointSelNum);
  Eigen::Matrix<float,6,6> matAtA;
  Eigen::VectorXf matB(pointSelNum);
  Eigen::Matrix<float,6,1> matAtB;
  Eigen::Matrix<float,6,1> matX;
  Eigen::Matrix<float,6,6> matP;

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float a1 = crx * sry * srz;
  float a2 = crx * crz * sry;
  float a3 = srx * sry;
  float a4 = tx * a1 - ty * a2 - tz * a3;
  float a5 = srx * srz;
  float a6 = crz * srx;
  float a7 = ty * a6 - tz * crx - tx * a5;
  float a8 = crx * cry * srz;
  float a9 = crx * cry * crz;
  float a10 = cry * srx;
  float a11 = tz * a10 + ty * a9 - tx * a8;

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b3 = crx * cry;
  float b4 = tx * -b1 + ty * -b2 + tz * b3;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;
  float b7 = crx * sry;
  float b8 = tz * b7 - ty * b6 - tx * b5;

  float c1 = -b6;
  float c2 = b5;
  float c3 = tx * b6 - ty * b5;
  float c4 = -crx * crz;
  float c5 = crx * srz;
  float c6 = ty * c5 + tx * -c4;
  float c7 = b2;
  float c8 = -b1;
  float c9 = tx * -b2 - ty * -b1;

  for (int i = 0; i < pointSelNum; i++) {
    PointType pointOri = laserCloudOri->points[i];
    PointType coeff = coeffSel->points[i];

    float arx =
        (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x +
        (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y +
        (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

    float ary =
        (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x +
        (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

    float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x +
                (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y +
                (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

    float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

    float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

    float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

    float d2 = coeff.intensity;

    matA(i, 0) = arx;
    matA(i, 1) = ary;
    matA(i, 2) = arz;
    matA(i, 3) = atx;
    matA(i, 4) = aty;
    matA(i, 5) = atz;
    matB(i, 0) = -0.05 * d2;
  }

  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1, 6> matE;
    Eigen::Matrix<float,6, 6> matV;
    Eigen::Matrix<float,6, 6> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6,6> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;

    isDegenerate = false;
    float eignThre[6] = {10, 10, 10, 10, 10, 10};
    for (int i = 5; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 6; j++) {
          matV2(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate) {
    Eigen::Matrix<float,6,1> matX2;
    matX2 = matX;
    matX = matP * matX2;
  }

  transformCur[0] += matX(0, 0);
  transformCur[1] += matX(1, 0);
  transformCur[2] += matX(2, 0);
  transformCur[3] += matX(3, 0);
  transformCur[4] += matX(4, 0);
  transformCur[5] += matX(5, 0);

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transformCur[i])) transformCur[i] = 0;
  }

  float deltaR = sqrt(pow(RAD2DEG * (matX(0, 0)), 2) +
                      pow(RAD2DEG * (matX(1, 0)), 2) +
                      pow(RAD2DEG * (matX(2, 0)), 2));
  float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                      pow(matX(4, 0) * 100, 2) +
                      pow(matX(5, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

void FeatureAssociation::checkSystemInitialization() {
  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;
  // 检查点云大小
  // std::cout << "checkSystemInitialization " << std::endl;
  // std::cout << "The point cloud without virtual points : " << laserCloudSurfLast->size() << std::endl;
  // ------------------------------------------------------------------------------------
  // Alex
  for (const auto& point : *virtualshadowCloud) {
      laserCloudSurfLast->push_back(point);
  }
  // std::cout << "The point cloud with virtual points : " << laserCloudSurfLast->size() << std::endl;
  // std::cout << "checkSystemInitialization " << std::endl;
  // ------------------------------------------------------------------------------------

  kdtreeCornerLast.setInputCloud(laserCloudCornerLast);
  kdtreeSurfLast.setInputCloud(laserCloudSurfLast);

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  sensor_msgs::msg::PointCloud2 laserCloudCornerLast2;
  pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
  laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
  laserCloudCornerLast2.header.frame_id = "camera";
  _pub_cloud_corner_last->publish(laserCloudCornerLast2);

  sensor_msgs::msg::PointCloud2 laserCloudSurfLast2;
  pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
  laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
  laserCloudSurfLast2.header.frame_id = "camera";
  _pub_cloud_surf_last->publish(laserCloudSurfLast2);

  if(use_imu_undistortion){
    transformSum[0] += imuPitchStart;
    transformSum[2] += imuRollStart;
  }

  systemInitedLM = true;
}

void FeatureAssociation::updateInitialGuess(){

    imuPitchLast = imuPitchCur;
    imuYawLast = imuYawCur;
    imuRollLast = imuRollCur;

    imuShiftFromStartX = imuShiftFromStartXCur;
    imuShiftFromStartY = imuShiftFromStartYCur;
    imuShiftFromStartZ = imuShiftFromStartZCur;

    imuVeloFromStartX = imuVeloFromStartXCur;
    imuVeloFromStartY = imuVeloFromStartYCur;
    imuVeloFromStartZ = imuVeloFromStartZCur;

    if (imuAngularFromStartX != 0 || imuAngularFromStartY != 0 || imuAngularFromStartZ != 0){
        transformCur[0] = - imuAngularFromStartY;
        transformCur[1] = - imuAngularFromStartZ;
        transformCur[2] = - imuAngularFromStartX;
    }
    
    if (imuVeloFromStartX != 0 || imuVeloFromStartY != 0 || imuVeloFromStartZ != 0){
        transformCur[3] -= imuVeloFromStartX * _scan_period;
        transformCur[4] -= imuVeloFromStartY * _scan_period;
        transformCur[5] -= imuVeloFromStartZ * _scan_period;
    }

    // //Alex

    // // RCLCPP_INFO(this->get_logger(), "pre compensation y: '%f'", transformCur[4]);
    // // RCLCPP_INFO(this->get_logger(), "pre compensation z: '%f'", transformCur[5]);
    // // RCLCPP_INFO(this->get_logger(), "pre compensation x: '%f'", transformCur[3]);
    // // RCLCPP_INFO(this->get_logger(), "IMU time: '%f'", imuTime[imuPointerLast]);

    // Eigen::Matrix3d rotation_matrix_now;
    // Eigen::Matrix3d rotation_matrix_last;
    // rotation_matrix_now = Eigen::AngleAxisd(odomYaw[odomPointerLast], Eigen::Vector3d::UnitZ()) *
    //                       Eigen::AngleAxisd(odomPitch[odomPointerLast], Eigen::Vector3d::UnitY()) *
    //                       Eigen::AngleAxisd(odomRoll[odomPointerLast], Eigen::Vector3d::UnitX());
    // // rotation_matrix_last = Eigen::AngleAxisd(-transformSum[1], Eigen::Vector3d::UnitY()) *
    // //                        Eigen::AngleAxisd(-transformSum[0], Eigen::Vector3d::UnitX()) *
    // //                        Eigen::AngleAxisd(transformSum[2], Eigen::Vector3d::UnitZ());
    // rotation_matrix_last = Eigen::AngleAxisd(transformSum[1], Eigen::Vector3d::UnitZ()) *
    //                        Eigen::AngleAxisd(transformSum[0], Eigen::Vector3d::UnitY()) *
    //                        Eigen::AngleAxisd(transformSum[2], Eigen::Vector3d::UnitX());
    // Eigen::Matrix3d rotationFromStartToEnd = rotation_matrix_last.transpose() * rotation_matrix_now;
    // rotationFromStartToEnd.transposeInPlace();


    // // without singular //https://blog.csdn.net/WillWinston/article/details/125746107
    // odomX = std::atan2(rotationFromStartToEnd(2, 1), rotationFromStartToEnd(2, 2)); //odomRoll
    // odomY = std::atan2(-rotationFromStartToEnd(2, 0), std::sqrt(rotationFromStartToEnd(2, 1) * rotationFromStartToEnd(2, 1) + rotationFromStartToEnd(2, 2) * rotationFromStartToEnd(2, 2)));  //odomPitch
    // odomZ = std::atan2(rotationFromStartToEnd(1, 0), rotationFromStartToEnd(0, 0)); //odomYaw
    
    // transformCur[0] = odomY; //odomRoll
    // transformCur[1] = odomZ; //odomPitch
    // transformCur[2] = odomX; //odomYaw

    // Eigen::Vector3d transVector;
    // Eigen::Vector3d outer_param_body;
    // outer_param_body << 0.08, 0, 0.0377;
    // Eigen::Vector3d outer_param_global = rotation_matrix_now * outer_param_body;
    // transVector << (transformSum[5] - (odomPosX[odomPointerLast] + outer_param_body[0] + outer_param_global[0])), (transformSum[3] - (odomPosY[odomPointerLast] + outer_param_body[1] + outer_param_global[1])), (transformSum[4] - (odomPosZ[odomPointerLast] + outer_param_body[2] + outer_param_global[2]));
    // Eigen::Vector3d transVectorBody = rotation_matrix_last.transpose() * transVector;
    // transformCur[5] = transVectorBody[0];
    // transformCur[3] = transVectorBody[1];
    // transformCur[4] = transVectorBody[2];


    // // // 打印矩陣
    // // std::cout << "Here is the matrix 3x3:\n" << a << std::endl;
    // // Eigen::Matrix3d rotationFromStartToEnd = rotation_matrix_now;
    // // singular                       
    // // Eigen::Vector3d eulerFromStartToEnd = rotation_matrix_now.eulerAngles(2, 1, 0);
    // // odomX = eulerFromStartToEnd[0];
    // // odomY = eulerFromStartToEnd[1];
    // // odomZ = eulerFromStartToEnd[2];
    // // odomZ = eulerFromStartToEnd[2];

    // // Eigen::Vector3d transVector;
    // // Eigen::Vector3d outer_param_body;
    // // outer_param_body << 0.08, 0, 0.0377;
    // // Eigen::Vector3d outer_param_global = rotation_matrix_now * outer_param_body;
    // // transVector << (transformSum[5] - (odomPosX[odomPointerLast] + outer_param_global[0])), (transformSum[3] - (odomPosY[odomPointerLast] + outer_param_global[1])), (transformSum[4] - (odomPosZ[odomPointerLast] + outer_param_global[2]));
    // // Eigen::Vector3d transVectorBody = rotation_matrix_now.transpose() * transVector;
    

    // // odomX = std::atan2(rotationFromStartToEnd(2, 1), rotationFromStartToEnd(2, 2)); //odomRoll
    // // odomY = std::atan2(-rotationFromStartToEnd(2, 0), std::sqrt(rotationFromStartToEnd(2, 1) * rotationFromStartToEnd(2, 1) + rotationFromStartToEnd(2, 2) * rotationFromStartToEnd(2, 2)));  //odomPitch
    // // odomZ = std::atan2(rotationFromStartToEnd(1, 0), rotationFromStartToEnd(0, 0)); //odomYaw
    // // transformCur[0] = odomY; //odomRoll
    // // transformCur[1] = odomZ; //odomPitch
    // // transformCur[2] = odomX; //odomYaw
    
    // // odomX = std::atan2(-rotationFromStartToEnd(1, 2), std::sqrt(rotationFromStartToEnd(1, 1) * rotationFromStartToEnd(1, 1) + rotationFromStartToEnd(1, 0) * rotationFromStartToEnd(1, 0))) - transformSum[0]; //odomRoll
    // // odomY = std::atan2(rotationFromStartToEnd(0, 2), rotationFromStartToEnd(2, 2)) - transformSum[1];  //odomPitch
    // // odomZ = std::atan2(rotationFromStartToEnd(0, 1), rotationFromStartToEnd(0, 0)) - transformSum[2]; //odomYaw
    // // transformCur[1] = std::atan2(-rotationFromStartToEnd(1, 2), 1); //odomRoll
    // // transformCur[2] = std::atan2(rotationFromStartToEnd(0, 2), rotationFromStartToEnd(2, 2));  //odomPitch
    // // transformCur[0] = std::atan2(rotationFromStartToEnd(0, 1), rotationFromStartToEnd(0, 0)); //odomYaw

    // // transformCur[0] = -(odomPitch[odomPointerLast] - transformSum[0]);
    // // transformCur[1] = -(odomYaw[odomPointerLast] - transformSum[1]);
    // // transformCur[2] = -(odomRoll[odomPointerLast] - transformSum[2]);
    // // transformCur[3] = odomPosY[odomPointerLast] - transformSum[3];
    // // transformCur[4] = odomPosZ[odomPointerLast] - transformSum[4];
    // // transformCur[5] = odomPosX[odomPointerLast] - transformSum[5];
    // // odomStartPosX = 0;odomStartPosY = 0;odomStartPosZ = 0;
    // // odomStartRoll = 0;odomStartPitch = 0;odomStartYaw = 0;

    // // RCLCPP_INFO(this->get_logger(), "odom x: '%f'", (odomY + imuAngularFromStartZ)*57.32);
    // // RCLCPP_INFO(this->get_logger(), "odom Y: '%f'", (odomZ + imuAngularFromStartX)*57.32);
    // // RCLCPP_INFO(this->get_logger(), "odom Z: '%f'", (odomX + imuAngularFromStartY)*57.32);
    // // RCLCPP_INFO(this->get_logger(), "slam X: '%f'", transformCur[0]);
    // // RCLCPP_INFO(this->get_logger(), "slam Y: '%f'", transformCur[1]);
    // // RCLCPP_INFO(this->get_logger(), "slam Z: '%f'", transformCur[2]);

    // // auto message2 = geometry_msgs::msg::Vector3();
    // // message2.x = odomY;
    // // message2.y = odomZ;
    // // message2.z = odomX;
    // // pubRotateMsgs->publish(message2);

    // // auto message2 = geometry_msgs::msg::Vector3();
    // // message2.x = transVector[0];
    // // message2.y = transVector[1];
    // // message2.z = transVector[2];
    // // pubRotateMsgs->publish(message2);
    // // transformCur[5] = transVectorBody[0];
    // // transformCur[3] = transVectorBody[1];
    // // transformCur[4] = transVectorBody[2];
    
    // // odomX = std::atan2(-rotationFromStartToEnd(1, 2), std::sqrt(rotationFromStartToEnd(1, 1) * rotationFromStartToEnd(1, 1) + rotationFromStartToEnd(1, 0) * rotationFromStartToEnd(1, 0))) - transformSum[0]; //odomRoll
    // // odomY = std::atan2(rotationFromStartToEnd(0, 2), rotationFromStartToEnd(2, 2)) - transformSum[1];  //odomPitch
    // // odomZ = std::atan2(rotationFromStartToEnd(0, 1), rotationFromStartToEnd(0, 0)) - transformSum[2]; //odomYaw
    // // transformCur[1] = std::atan2(-rotationFromStartToEnd(1, 2), 1); //odomRoll
    // // transformCur[2] = std::atan2(rotationFromStartToEnd(0, 2), rotationFromStartToEnd(2, 2));  //odomPitch
    // // transformCur[0] = std::atan2(rotationFromStartToEnd(0, 1), rotationFromStartToEnd(0, 0)); //odomYaw

    // // transformCur[0] = -(odomPitch[odomPointerLast] - transformSum[0]);
    // // transformCur[1] = -(odomYaw[odomPointerLast] - transformSum[1]);
    // // transformCur[2] = -(odomRoll[odomPointerLast] - transformSum[2]);
    // // transformCur[3] = odomPosY[odomPointerLast] - transformSum[3];
    // // transformCur[4] = odomPosZ[odomPointerLast] - transformSum[4];
    // // transformCur[5] = odomPosX[odomPointerLast] - transformSum[5];
    // // odomStartPosX = 0;odomStartPosY = 0;odomStartPosZ = 0;
    // // odomStartRoll = 0;odomStartPitch = 0;odomStartYaw = 0;

    // // RCLCPP_INFO(this->get_logger(), "odom x: '%f'", (odomY + imuAngularFromStartZ)*57.32);
    // // RCLCPP_INFO(this->get_logger(), "odom Y: '%f'", (odomZ + imuAngularFromStartX)*57.32);
    // // RCLCPP_INFO(this->get_logger(), "odom Z: '%f'", (odomX + imuAngularFromStartY)*57.32);
    // // RCLCPP_INFO(this->get_logger(), "slam X: '%f'", outer_param_global[0]);
    // // RCLCPP_INFO(this->get_logger(), "slam Y: '%f'", outer_param_global[1]);
    // // RCLCPP_INFO(this->get_logger(), "slam Z: '%f'", outer_param_global[2]);

    // // auto message2 = geometry_msgs::msg::Vector3();
    // // message2.x = odomY;
    // // message2.y = odomZ;
    // // message2.z = odomX;
    // // pubRotateMsgs->publish(message2);

    // auto message2 = geometry_msgs::msg::Vector3();
    // message2.x = odomYaw[odomPointerLast];
    // message2.y = odomPitch[odomPointerLast];
    // message2.z = odomRoll[odomPointerLast];
    // pubRotateMsgs->publish(message2);

}

void FeatureAssociation::updateTransformation() {
  if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100) return;

  for (int iterCount1 = 0; iterCount1 < 100; iterCount1++) {
    laserCloudOri->clear();
    coeffSel->clear();
    // RCLCPP_INFO(this->get_logger(), "PointSize: %d", laserCloudOri->points.size()); //Alex debug
    findCorrespondingSurfFeatures(iterCount1);
    // RCLCPP_INFO(this->get_logger(), "PointSize: %d", laserCloudOri->points.size()); //Alex debug
    // RCLCPP_INFO(this->get_logger(), "SurfScanRegistrationIterTimes: %d", iterCount1); //Alex debug
    if (laserCloudOri->points.size() < 10) continue;
    if (calculateTransformationSurf(iterCount1) == false) break;
  }
  // RCLCPP_INFO(this->get_logger(), "Your log message here"); //Alex debug

  for (iterCount2 = 0; iterCount2 < 100; iterCount2++) {
    laserCloudOri->clear();
    coeffSel->clear();
    // RCLCPP_INFO(this->get_logger(), "PointSize: %d", laserCloudOri->points.size()); //Alex debug
    findCorrespondingCornerFeatures(iterCount2);
    // RCLCPP_INFO(this->get_logger(), "PointSize: %d", laserCloudOri->points.size()); //Alex debug
    // RCLCPP_INFO(this->get_logger(), "CornerScanRegistrationIterTimes: %d", iterCount2); //Alex debug
    if (laserCloudOri->points.size() < 10) continue;
    if (calculateTransformationCorner(iterCount2) == false) break;
  }
  // auto message = geometry_msgs::msg::Vector3();
  // message.x = transformCur[0];
  // message.y = transformCur[1];
  // message.z = transformCur[2];
  // pubRotateMsgs->publish(message);
}

void FeatureAssociation::integrateTransformation() {
  float rx, ry, rz, tx, ty, tz;
  AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
                     -transformCur[0], -transformCur[1], -transformCur[2], rx,
                     ry, rz);

  float x1 = cos(rz) * (transformCur[3] ) -
             sin(rz) * (transformCur[4] );
  float y1 = sin(rz) * (transformCur[3] ) +
             cos(rz) * (transformCur[4] );
  float z1 = transformCur[5];

  float x2 = x1;
  float y2 = cos(rx) * y1 - sin(rx) * z1;
  float z2 = sin(rx) * y1 + cos(rx) * z1;

  tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
  ty = transformSum[4] - y2;
  tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

  if(use_imu_undistortion){
    PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart, 
                        imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);
  }

  transformSum[0] = rx;
  transformSum[1] = ry;
  transformSum[2] = rz;
  transformSum[3] = tx;
  transformSum[4] = ty;
  transformSum[5] = tz;

  // //Alex
  // Eigen::Matrix3d rotation_matrix_now;
  // Eigen::Matrix3d rotation_matrix_last;
  // rotation_matrix_now = Eigen::AngleAxisd(odomYaw[odomPointerLast], Eigen::Vector3d::UnitZ()) *
  //                       Eigen::AngleAxisd(odomPitch[odomPointerLast], Eigen::Vector3d::UnitY()) *
  //                       Eigen::AngleAxisd(odomRoll[odomPointerLast], Eigen::Vector3d::UnitX());
  // // rotation_matrix_last = Eigen::AngleAxisd(transformSum[1], Eigen::Vector3d::UnitY()) *
  // //                        Eigen::AngleAxisd(transformSum[0], Eigen::Vector3d::UnitX()) *
  // //                        Eigen::AngleAxisd(transformSum[2], Eigen::Vector3d::UnitZ());
  // rotation_matrix_last = Eigen::AngleAxisd(transformSum[1], Eigen::Vector3d::UnitZ()) *
  //                        Eigen::AngleAxisd(transformSum[0], Eigen::Vector3d::UnitY()) *
  //                        Eigen::AngleAxisd(transformSum[2], Eigen::Vector3d::UnitX());
  // Eigen::Matrix3d rotationFromStartToEnd = rotation_matrix_last;
  //   // singular                       
  //   // Eigen::Vector3d eulerFromStartToEnd = rotation_matrix_now.eulerAngles(2, 1, 0);
  //   // odomX = eulerFromStartToEnd[0];
  //   // odomY = eulerFromStartToEnd[1];
  //   // odomZ = eulerFromStartToEnd[2];

  //   // without singular //https://blog.csdn.net/WillWinston/article/details/125746107
  //   odomX = std::atan2(rotationFromStartToEnd(2, 1), rotationFromStartToEnd(2, 2)); //odomRoll
  //   odomY = std::atan2(-rotationFromStartToEnd(2, 0), std::sqrt(rotationFromStartToEnd(2, 1) * rotationFromStartToEnd(2, 1) + rotationFromStartToEnd(2, 2) * rotationFromStartToEnd(2, 2)));  //odomPitch
  //   odomZ = std::atan2(rotationFromStartToEnd(1, 0), rotationFromStartToEnd(0, 0)); //odomYaw 

  // auto message2 = geometry_msgs::msg::Vector3();
  //   message2.x = odomY - odomPitch[odomPointerLast];
  //   message2.y = odomZ - odomYaw[odomPointerLast];
  //   message2.z = odomX - odomRoll[odomPointerLast];
  //   pubRotateMsgs->publish(message2);
}

void FeatureAssociation::adjustOutlierCloud() {
  PointType point;
  int cloudSize = outlierCloud->points.size();
  for (int i = 0; i < cloudSize; ++i) {
    point.x = outlierCloud->points[i].y;
    point.y = outlierCloud->points[i].z;
    point.z = outlierCloud->points[i].x;
    point.intensity = outlierCloud->points[i].intensity;
    outlierCloud->points[i] = point;
  }
}

void FeatureAssociation::publishOdometry() {
  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion geoQuat;
  q.setRPY(transformSum[2], -transformSum[0], -transformSum[1]);
  geoQuat = tf2::toMsg(q);

  laserOdometry.header.stamp = cloudHeader.stamp;
  laserOdometry.pose.pose.orientation.x = -geoQuat.y;
  laserOdometry.pose.pose.orientation.y = -geoQuat.z;
  laserOdometry.pose.pose.orientation.z = geoQuat.x;
  laserOdometry.pose.pose.orientation.w = geoQuat.w;
  laserOdometry.pose.pose.position.x = transformSum[3];
  laserOdometry.pose.pose.position.y = transformSum[4];
  laserOdometry.pose.pose.position.z = transformSum[5];
  pubLaserOdometry->publish(laserOdometry);

  laserOdometryTrans.header.stamp = cloudHeader.stamp;
  laserOdometryTrans.transform.translation.x = transformSum[3];
  laserOdometryTrans.transform.translation.y = transformSum[4];
  laserOdometryTrans.transform.translation.z = transformSum[5];
  laserOdometryTrans.transform.rotation.x = -geoQuat.y;
  laserOdometryTrans.transform.rotation.y = -geoQuat.z;
  laserOdometryTrans.transform.rotation.z = geoQuat.x;
  laserOdometryTrans.transform.rotation.w = geoQuat.w;
  tfBroadcaster->sendTransform(laserOdometryTrans);
}

void FeatureAssociation::publishCloud() {
  sensor_msgs::msg::PointCloud2 laserCloudOutMsg;

  auto Publish = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                     const pcl::PointCloud<PointType>::Ptr &cloud) {
    if (pub->get_subscription_count() != 0) {
      pcl::toROSMsg(*cloud, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp = cloudHeader.stamp;
      laserCloudOutMsg.header.frame_id = "camera";
      pub->publish(laserCloudOutMsg);
    }
  };

  Publish(pubCornerPointsSharp, cornerPointsSharp);
  Publish(pubCornerPointsLessSharp, cornerPointsLessSharp);
  Publish(pubSurfPointsFlat, surfPointsFlat);
  Publish(pubSurfPointsLessFlat, surfPointsLessFlat);
  Publish(pubSegmentedCloud, segmentedCloud);
  Publish(pubDistortedCloud, distortedCloud);
}

void FeatureAssociation::publishCloudsLast() {

  if(use_imu_undistortion){
    updateImuRollPitchYawStartSinCos();
  }

  int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
  for (int i = 0; i < cornerPointsLessSharpNum; i++) {
    TransformToEnd(&cornerPointsLessSharp->points[i],
                   &cornerPointsLessSharp->points[i]);
  }

  int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
  for (int i = 0; i < surfPointsLessFlatNum; i++) {
    TransformToEnd(&surfPointsLessFlat->points[i],
                   &surfPointsLessFlat->points[i]);
  }

  int cornerPointsSharpNum = cornerPointsSharp->points.size();
  for (int i = 0; i < cornerPointsSharpNum; i++) {
    TransformToEnd(&cornerPointsSharp->points[i],
                   &cornerPointsSharp->points[i]);
  }

  int surfPointsFlatNum = surfPointsFlat->points.size();
  // std::cout << "surfPointsFlatNum : " << surfPointsFlatNum << std::endl;
  for (int i = 0; i < surfPointsFlatNum; i++) {
    TransformToEnd(&surfPointsFlat->points[i],
                   &surfPointsFlat->points[i]);
  }

  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  laserCloudCornerScan = cornerPointsSharp;
  laserCloudSurfScan = surfPointsFlat;
  // 检查点云大小
  // std::cout << "The point cloud without virtual points : " << laserCloudSurfLast->size() << std::endl;
  // ------------------------------------------------------------------------------------
  // Alex
  for (const auto& point : *virtualshadowCloud) {
      laserCloudSurfLast->push_back(point);
  }
  // std::cout << "The point cloud with virtual points : " << laserCloudSurfLast->size() << std::endl;
  // ------------------------------------------------------------------------------------

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
    kdtreeCornerLast.setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast.setInputCloud(laserCloudSurfLast);
  }

  frameCount++;
  adjustOutlierCloud(); // to loam coordinate

  if (frameCount >= skipFrameNum + 1) {
    frameCount = 0;
    sensor_msgs::msg::PointCloud2 cloudTemp;

    auto Publish = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                       const pcl::PointCloud<PointType>::Ptr &cloud) {
      if (pub->get_subscription_count() != 0) {
        pcl::toROSMsg(*cloud, cloudTemp);
        cloudTemp.header.stamp = cloudHeader.stamp;
        cloudTemp.header.frame_id = "camera";
        pub->publish(cloudTemp);
      }
    };

    Publish(_pub_outlier_cloudLast, outlierCloud);
    Publish(_pub_cloud_corner_last, laserCloudCornerLast);
    Publish(_pub_cloud_surf_last, laserCloudSurfLast);
  }
}

void FeatureAssociation::runFeatureAssociation() {
  while (rclcpp::ok()) {
    ProjectionOut projection;
    _input_channel.receive(projection);

    if( !rclcpp::ok() ) break;

    //--------------
    frame_idx2++;
    outlierCloud = projection.outlier_cloud;
    segmentedCloud = projection.segmented_cloud;
    outlierCloudIntensity = projection.outlierCloud_Intensity;
    segmentedCloudIntensity = projection.segmentedCloud_Intensity;
    // visualCloud = projection.visual_cloud;
    // fullCloud = projection.full_cloud;
    segInfo = std::move(projection.seg_msg);

    cloudHeader = segInfo.header;

    timeScanCur = cloudHeader.stamp.sec + cloudHeader.stamp.nanosec/1e9;
    // std::cout << cloudHeader.stamp.nanosec << std::endl ;
    std::chrono::steady_clock::time_point time_start2 = std::chrono::steady_clock::now();

    // Alex for Shadow Virtual Feature Point
    GenerateShadowPoint();

    /**  1. Feature Extraction  */
    adjustDistortion(); // to loam coordinate

    calculateSmoothnessOurs();

    markOccludedPoints();

    extractFeaturesOurs();
    // extractFeatures();

    publishCloud();  // cloud for visualization

    // Feature Association
    if (!systemInitedLM) {
      checkSystemInitialization();
      continue;
    }

    if (use_imu_undistortion){
      updateInitialGuess();
    }
    updateTransformation();

    integrateTransformation();

    publishOdometry();

    publishCloudsLast();  // cloud to mapOptimization

    // std::cout << "Frame Index FA:" << frame_idx2 << std::endl;
    double gseg_runtime_all2 = 0;
    std::chrono::steady_clock::time_point time_end2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds2 = time_end2 - time_start2;
    double runtime_frame2 = elapsed_seconds2.count() * 1000;
    gseg_runtime_list2.push_back(runtime_frame2/1000);
    for (size_t i = 0; i<gseg_runtime_list2.size(); ++i){
      gseg_runtime_all2 += gseg_runtime_list2[i];
    }
    float gseg_Hz2 = (gseg_runtime_all2/gseg_runtime_list2.size())*1000;
    // std::cout << "Feature Association Runtime (ms):" << gseg_Hz2 << std::endl; //Alex debug

    double odometry_itertimes_all = 0;
    odometry_itertimes_list.push_back(iterCount2);
    for (size_t i = 0; i<odometry_itertimes_list.size(); ++i){
      odometry_itertimes_all += odometry_itertimes_list[i];
    }
    float odometry_itertimes = (odometry_itertimes_all/odometry_itertimes_list.size());
    // std::cout << "Odometry Iteration Times:" << odometry_itertimes << std::endl; // Bii  //Alex debug
    //--------------
    _cycle_count++;

    if (static_cast<int>(_cycle_count) == _mapping_frequency_div) {
      _cycle_count = 0;
      AssociationOut out;
      out.cloud_corner_last.reset(new pcl::PointCloud<PointType>());
      out.cloud_surf_last.reset(new pcl::PointCloud<PointType>());
      out.cloud_outlier_last.reset(new pcl::PointCloud<PointType>());
      out.cloud_corner_scan.reset(new pcl::PointCloud<PointType>());
      out.cloud_surf_scan.reset(new pcl::PointCloud<PointType>());

      *out.cloud_corner_last = *laserCloudCornerLast;
      *out.cloud_surf_last = *laserCloudSurfLast;
      *out.cloud_corner_scan = *laserCloudCornerScan;
      *out.cloud_surf_scan = *laserCloudSurfScan;
      *out.cloud_outlier_last = *outlierCloud;

      out.laser_odometry = laserOdometry;

      // sensor_msgs::msg::PointCloud2 cloudTemp;
      // auto Publish = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
      //                   const pcl::PointCloud<PointType>::Ptr &cloud) {
      //   if (pub->get_subscription_count() != 0) {
      //     pcl::toROSMsg(*cloud, cloudTemp);
      //     cloudTemp.header.stamp = cloudHeader.stamp;
      //     cloudTemp.header.frame_id = "camera";
      //     pub->publish(cloudTemp);
      //   }
      // };

      // Publish(_pub_visual_cloud, visualCloud);

      _output_channel.send(std::move(out));
    }
  }
}
