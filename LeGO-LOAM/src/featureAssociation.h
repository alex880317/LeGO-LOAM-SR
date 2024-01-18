#ifndef FEATUREASSOCIATION_H
#define FEATUREASSOCIATION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include "lego_loam/nanoflann_pcl.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/Core> // bii
#include <Eigen/Geometry> // bii
#include <iostream> // bii
#include <algorithm>
#include "lego_loam/tictoc.h"
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>  //Alex
#include <sensor_msgs/msg/joint_state.hpp>  //Alex
#include <geometry_msgs/msg/vector3.hpp>  //Alex
// /home/iec/LeGOMapping_ws/src/neptune_loam_localizer/LeGO-LOAM/src/featureAssociation.h
class FeatureAssociation : public rclcpp::Node {

 public:
  FeatureAssociation(const std::string &name, Channel<ProjectionOut>& input_channel,
                     Channel<AssociationOut>& output_channel);

  ~FeatureAssociation();

  void runFeatureAssociation();
  void visualcloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

 private:

  int _vertical_scans;
  int _horizontal_scans;
  float _scan_period;
  float _edge_threshold;
  float _surf_threshold;
  float _nearest_feature_dist_sqr;
  int _mapping_frequency_div;
  float _ang_bottom;
  float vertical_angle_top;
  float _ang_resolution_X;
  float _ang_resolution_Y;
  float DBFr;
  float RatioXY;
  float RatioZ;
  int frame_idx2 = 0;
  double accumulated_run_time2 = 0;


  std::thread _run_thread;


  bool use_imu_undistortion;

  double timeScanCur;
  double timeNewSegmentedCloud;
  double timeNewSegmentedCloudInfo;
  double timeNewOutlierCloud;

  bool newSegmentedCloud;
  bool newSegmentedCloudInfo;
  bool newOutlierCloud;
  
  int imuQueLength = 200;
  int imuPointerFront;
  int imuPointerLast;
  int imuPointerLastIteration;

  float imuRollStart, imuPitchStart, imuYawStart;
  float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart;
  float imuRollCur, imuPitchCur, imuYawCur;

  float imuVeloXStart, imuVeloYStart, imuVeloZStart;
  float imuShiftXStart, imuShiftYStart, imuShiftZStart;

  float imuVeloXCur, imuVeloYCur, imuVeloZCur;
  float imuShiftXCur, imuShiftYCur, imuShiftZCur;

  float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
  float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

  float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur;
  float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast;
  float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ;

  double imuTime[200];
  float imuRoll[200];
  float imuPitch[200];
  float imuYaw[200];
  float imuRollPreSurf;
  float imuPitchPreSurf;
  float imuYawPreSurf;
  float imuRollPreCorner;
  float imuPitchPreCorner;
  float imuYawPreCorner;
  
  float imuAccX[200];
  float imuAccY[200];
  float imuAccZ[200];

  float imuVeloX[200];
  float imuVeloY[200];
  float imuVeloZ[200];

  float imuShiftX[200];
  float imuShiftY[200];
  float imuShiftZ[200];

  float imuAngularVeloX[200];
  float imuAngularVeloY[200];
  float imuAngularVeloZ[200];

  float imuAngularRotationX[200];
  float imuAngularRotationY[200];
  float imuAngularRotationZ[200];


  float imuRollLast, imuPitchLast, imuYawLast;
  float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
  float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

  //Alex
  int odomPointerLast;
  int odomQueLength = 200;
  double odomTime[200];
  float odomRoll[200];
  float odomPitch[200];
  float odomYaw[200];
  float odomStartPosX;float odomStartPosY;float odomStartPosZ;
  float odomStartRoll;float odomStartPitch;float odomStartYaw;

  float odomPosX[200]; 
  float odomPosY[200]; 
  float odomPosZ[200];
  float odomX;
  float odomY;
  float odomZ;
  // float TurnRF[200];
  


  //Alex
  int encoderQueLength = 200;
  double encoderTime[200];
  float TurnRF[200];
  float WheelRR[200];
  float WheelLR[200];
  float Wheelk[200];
  


  int iterCount2;


  Channel<ProjectionOut>& _input_channel;
  Channel<AssociationOut>& _output_channel;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_visual_cloud;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom_gt;  //Alex
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_Ack_encoder;  //Alex
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubRotateMsgs;  //Alex

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsLessSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsFlat;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsLessFlat;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSegmentedCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDistortedCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_visual_cloud;

  pcl::PointCloud<PointType>::Ptr distortedCloud;
  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr visualCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;
  pcl::PointCloud<PointType>::Ptr fullCloud;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;

  cloud_msgs::msg::CloudInfo segInfo;
  std_msgs::msg::Header cloudHeader;

  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float> cloudCurvature;
  std::vector<float> kxy;
  std::vector<float> kz;
  std::vector<int> cloudNeighborPicked;
  std::vector<int> cloudNeighborPickedPlane;
  std::vector<int> cloudLabel;
  std::vector<int> cluster;
  std::vector<int> cluster_length;
  std::vector<int> cluster_num_list;
  std::vector<int> cluster_inlier;
  std::vector<int> in_idx;
  std::vector<int> in_label_list;
  std::vector<double> gseg_runtime_list2;
  std::vector<double> odometry_itertimes_list; // Bii
  std::vector<double> outlierCloudIntensity;
  std::vector<double> segmentedCloudIntensity;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud_corner_last;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud_surf_last;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloudLast;

  int skipFrameNum;
  bool systemInitedLM;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  std::vector<float> pointSearchCornerInd1;
  std::vector<float> pointSearchCornerInd2;

  std::vector<float> pointSearchSurfInd1;
  std::vector<float> pointSearchSurfInd2;
  std::vector<float> pointSearchSurfInd3;

  float transformCur[6];
  float transformSum[6];

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerScan;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfScan;
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerLast;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  nav_msgs::msg::Odometry laserOdometry;

  geometry_msgs::msg::TransformStamped laserOdometryTrans;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  
  bool isDegenerate;

  int frameCount;
  size_t _cycle_count;

 private:
  void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr imuIn);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void initializationValue();
  void AccumulateIMUShiftAndRotation();
  void updateImuRollPitchYawStartSinCos();
  void ShiftToStartIMU(float pointTime);
  void VeloToStartIMU();
  void TransformToStartIMU(PointType *p);
  void adjustDistortion();
  void calculateSmoothness();
  void calculateSmoothnessOurs();
  void markOccludedPoints();
  void extractFeatures();
  void extractFeaturesOurs();
  void DBSCAN_EdgeFeature();

  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);
  void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
                           float alx, float aly, float alz, float &acx, float &acy, float &acz);

  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly,
                          float lz, float &ox, float &oy, float &oz);

  void findCorrespondingCornerFeatures(int iterCount);
  void findCorrespondingSurfFeatures(int iterCount);

  bool calculateTransformationSurf(int iterCount);
  bool calculateTransformationCorner(int iterCount);
  bool calculateTransformation(int iterCount);

  void checkSystemInitialization();
  void checkSystemInitializationOurs();
  void updateInitialGuess();
  void updateTransformation();

  void integrateTransformation();
  void publishCloud();
  void publishOdometry();

  void adjustOutlierCloud();
  void publishCloudsLast();
  void publishCloudsLastOurs();

};

#endif // FEATUREASSOCIATION_H
