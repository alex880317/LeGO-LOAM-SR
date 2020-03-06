#ifndef TRANSFORMFUSION_H
#define TRANSFORMFUSION_H

#include "lego_loam/utility.h"

class TransformFusion {
 private:
  ros::NodeHandle& nh;
  ros::Publisher pubLaserOdometry2;
  ros::Subscriber subLaserOdometry;
  ros::Subscriber subOdomAftMapped;

  nav_msgs::Odometry laserOdometry2;
  geometry_msgs::TransformStamped laserOdometryTrans2;
  tf2_ros::TransformBroadcaster tfBroadcaster2;

  geometry_msgs::TransformStamped map_2_camera_init_Trans;
  tf2_ros::TransformBroadcaster tfBroadcasterMap2CameraInit;

  geometry_msgs::TransformStamped camera_2_base_link_Trans;
  tf2_ros::TransformBroadcaster tfBroadcasterCamera2Baselink;

  float transformSum[6];
  float transformIncre[6];
  float transformMapped[6];
  float transformBefMapped[6];
  float transformAftMapped[6];

  std_msgs::Header currentHeader;

 public:
  TransformFusion(ros::NodeHandle& node);

  void transformAssociateToMap();
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);
  void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped);
};




#endif // TRANSFORMFUSION_H
