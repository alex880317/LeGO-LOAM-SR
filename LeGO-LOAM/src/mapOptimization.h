#ifndef MAPOPTIMIZATION_H
#define MAPOPTIMIZATION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include "lego_loam/nanoflann_pcl.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include "lego_loam/tictoc.h"
#include <stdio.h>
#include <termios.h>
#include <iostream>
#include <string>
#include <vector>

inline gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
  // camera frame to lidar frame
  return gtsam::Pose3(
      gtsam::Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll),
                          double(thisPoint.pitch)),
      gtsam::Point3(double(thisPoint.z), double(thisPoint.x),
                    double(thisPoint.y)));
}

inline Eigen::Affine3f pclPointToAffine3fCameraToLidar(
    PointTypePose thisPoint) {
  // camera frame to lidar frame
  return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y,
                                thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
}

inline void mapLidarFrameToLoamFrame(pcl::PointCloud<PointType>::Ptr  inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
{
	Eigen::Affine3f transPose = Eigen::Translation3f(0, 0, 0)
														* Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX())
														* Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY())
														* Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ());
  pcl::transformPointCloud(*inputCloud, *outputCloud, transPose);
}

inline void mapLoamFrameToLiDARFrame(pcl::PointCloud<PointType>::Ptr  inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
{
	Eigen::Affine3f transPose = Eigen::Translation3f(0, 0, 0)
														* Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ())
														* Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitY())
														* Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX());
  pcl::transformPointCloud(*inputCloud, *outputCloud, transPose);
}


class MapOptimization : public rclcpp::Node {

 public:
  MapOptimization(const std::string &name, Channel<AssociationOut> &input_channel);

  ~MapOptimization();

  void run();
  void cloudHandlerMap(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
  void visualcloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

 private:

  gtsam::NonlinearFactorGraph gtSAMgraph;
  gtsam::Values initialEstimate;
  gtsam::ISAM2 *isam;
  gtsam::Values isamCurrentEstimate;

  bool _loop_closure_enabled;
  bool _save_key_feature_pcd;
  std::string fileSaveDirectory;
  float _surrounding_keyframe_search_radius;
  int   _surrounding_keyframe_search_num;
  float _history_keyframe_search_radius;
  int   _history_keyframe_search_num;
  float _history_keyframe_fitness_score;
  float _global_map_visualization_search_radius;
  int mapping_count;
  std::vector <double> time;

  pcl::PointCloud<PointType>::Ptr _laser_cloud_input;
  pcl::PointCloud<PointType>::Ptr visualCloud;

//   rclcpp::Service<lego_loam_sr::srv::SaveMap>::SharedPtr srvSaveMap;

  Channel<AssociationOut>& _input_channel;
  std::thread _run_thread;

  Channel<bool> _publish_global_signal;
  std::thread _publish_global_thread;
  void publishGlobalMapThread();

  Channel<bool> _loop_closure_signal;
  std::thread _loop_closure_thread;
  void loopClosureThread();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_laser_cloud_map;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_visual_cloud;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subSaveMap;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subInitalPose;
  void saveMapService(
    const geometry_msgs::msg::Twist::SharedPtr msg);
  void callbackInitalPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgeFeatureMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPlanarFeatureMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgeFeatureRegistered;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPlanarFeatureRegistered;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFramesFalse;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;

  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::TransformStamped aftMappedTrans;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

  std::vector<pcl::PointCloud<PointType>::Ptr> mapCloudKeyFrames;

  std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> visualCloudList;

  std::deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
  int latestFrameID;

  std::vector<int> surroundingExistingKeyPosesID;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;

  PointType previousRobotPosPoint;
  PointType currentRobotPosPoint;

  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  pcl::PointCloud<PointTypePose>::Ptr lidarKeyPoses6D;

  pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
  pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;

  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerLast;  // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfLast;  // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerLastDS;  // downsampled corner featuer set from
      // odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfLastDS;  // downsampled surf featuer set from
      // odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerScan;  // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfScan;  // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerScanDS;  // downsampled corner featuer set from
      // odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfScanDS;  // downsampled surf featuer set from
      // odoOptimization

  pcl::PointCloud<PointType>::Ptr
      laserCloudOutlierLast;  // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudOutlierLastDS;  // corner feature set from odoOptimization

  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfTotalLast;  // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfTotalLastDS;  // downsampled corner featuer set from
      // odoOptimization

  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerFromMap;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfFromMap;

  nanoflann::KdTreeFLANN<PointType> kdtreeSurroundingKeyPoses;
  nanoflann::KdTreeFLANN<PointType> kdtreeHistoryKeyPoses;

  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloudDS;

  pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

  nanoflann::KdTreeFLANN<PointType> kdtreeGlobalMap;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;
  pcl::PointCloud<PointType>::Ptr edgeMapKeyFrames;
  pcl::PointCloud<PointType>::Ptr edgeMapKeyFramesDS;
  pcl::PointCloud<PointType>::Ptr planarMapKeyFrames;
  pcl::PointCloud<PointType>::Ptr registeredEdgeFeature;
  pcl::PointCloud<PointType>::Ptr registeredPlanarFeature;
  pcl::PointCloud<PointType>::Ptr localInfo;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  std::vector<float> loop_icp_score;

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterOutlier;
  pcl::VoxelGrid<PointType>
      downSizeFilterHistoryKeyFrames;  // for histor key frames of loop closure
  pcl::VoxelGrid<PointType>
      downSizeFilterSurroundingKeyPoses;  // for surrounding key poses of
      // scan-to-map optimization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyPoses;  // for global map visualization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyFrames;  // for global map visualization

  rclcpp::Time timeLaserOdometry;

  float transformLast[6];
  float transformSum[6];
  float transformIncre[6];
  float transformTobeMapped[6];
  float transformBefMapped[6];
  float transformAftMapped[6];
  float CF_all;
  float CF_mean;
  float min_lambda;
  int frame_idx;
  int loop_idx;
  bool newInitalPoseFlag;
  double period_frame_pre = 0;
  std::vector<double> gseg_runtime_list3;
  std::vector<double> timestamp_diff;
  std::vector<double> MapIterTimes; // Bii
  int _iter_count_thres;
  float step_size;
  float stop_thres;
  std::chrono::steady_clock::time_point time_pre;

  std::mutex mtx;

  PointType pointOri, pointSel, pointProj, coeff;

  Eigen::Matrix<float, 5, 3> matA0;
  Eigen::Matrix<float, 5, 1> matB0;
  Eigen::Vector3f matX0;
  Eigen::Matrix<float, 1, 6> Pose6DOF;

  Eigen::Matrix3f matA1;
  Eigen::Matrix<float, 1, 3> matD1;
  Eigen::Matrix3f matV1;

  Eigen::Matrix<float, 6, 6> matP;

  bool isDegenerate;

  int laserCloudCornerFromMapDSNum;
  int laserCloudSurfFromMapDSNum;
  int laserCloudCornerLastDSNum;
  int laserCloudSurfLastDSNum;
  int laserCloudCornerScanDSNum;
  int laserCloudSurfScanDSNum;
  int laserCloudOutlierLastDSNum;
  int laserCloudSurfTotalLastDSNum;
  int frame_idx3 = 0;
//   double accumulated_run_time3 = 0;
  std::vector<double> accumulated_run_time3;

  bool potentialLoopFlag;
  int closestHistoryFrameID;
  int latestFrameIDLoopCloure;
  int latestFrameIDRegisteredFeature;
  int iter_num;

  bool aLoopIsClosed;

  float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
  float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

 private:
  void allocateMemory();
  void transformAssociateToMap();
  void transformUpdate();
  void updatePointAssociateToMapSinCos();
  void pointAssociateToMap(PointType const *const pi, PointType *const po);
  void updateTransformPointCloudSinCos(PointTypePose *tIn) ;

  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      pcl::PointCloud<PointType>::Ptr cloudIn);

  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

  void publishTF();
  void publishKeyPosesAndFrames();
  void publishGlobalMap();

  bool detectLoopClosure();
  void performLoopClosure();

  void extractSurroundingKeyFrames();
  void downsampleCurrentScan();
  void cornerOptimization();
  void surfOptimization();

  bool LMOptimization(int iterCount);
  void scan2MapOptimization();

  void saveKeyFramesAndFactor();
  void correctPoses();

  void clearCloud();
  void savePcd();
  void savePose();
  void savePcdInfo();
};

#endif // MAPOPTIMIZATION_H
