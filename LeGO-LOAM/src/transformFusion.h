#ifndef TRANSFORMFUSION_H
#define TRANSFORMFUSION_H

#include "lego_loam/utility.h"

class TransformFusion : public rclcpp::Node {
 private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry2;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdomAftMapped;

  nav_msgs::msg::Odometry laserOdometry2;
  geometry_msgs::msg::TransformStamped laserOdometryTrans2;
  tf2_ros::TransformBroadcaster tfBroadcaster2;

  float transformSum[6];
  float transformIncre[6];
  float transformMapped[6];
  float transformBefMapped[6];
  float transformAftMapped[6];

 public:
  TransformFusion(const std::string &name);

  void transformAssociateToMap();
  void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstPtr& laserOdometry);
  void odomAftMappedHandler(const nav_msgs::msg::Odometry::ConstPtr& odomAftMapped);
};




#endif // TRANSFORMFUSION_H
