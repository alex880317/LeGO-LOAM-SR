#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include <Eigen/QR>
#include "GRANSAC.hpp"
#include "PlaneModel.hpp"
#include <omp.h>
#include <opencv2/opencv.hpp>

class ImageProjection : public rclcpp::Node {
 public:

  ImageProjection(const std::string &name, Channel<ProjectionOut>& output_channel);

  ~ImageProjection() = default;

  void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

  // 定義點結構
  struct Point {
    double x, y, z;
    int index;

    // 構造函數，用於初始化 x, y, z 和 index
    Point(double x_val, double y_val, double z_val, int idx)
        : x(x_val), y(y_val), z(z_val), index(idx) {}
  };

 private:
  void findStartEndAngle();
  void resetParameters();
  void projectPointCloud();
  void groundRemoval();
  void cloudSegmentation();
  void labelComponents(int row, int col);
  void publishClouds();
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> ConvertPointCloudToGRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                                      const std::vector<int>& original_indices = std::vector<int>());
  double calculateDistance(const std::vector<double>& p, const std::vector<double>& Gk);
  double dotProduct(const std::vector<double>& vec1, const std::vector<double>& vec2);

  pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

  pcl::PointCloud<PointType>::Ptr _full_cloud;
  pcl::PointCloud<PointType>::Ptr _full_info_cloud;

  pcl::PointCloud<PointType>::Ptr _ground_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
  pcl::PointCloud<PointType>::Ptr _outlier_cloud;

  int _vertical_scans;
  int _horizontal_scans;
  float _ang_bottom;
  float _ang_resolution_X;
  float _ang_resolution_Y;
  float _segment_theta;
  int _segment_valid_point_num;
  int _segment_valid_line_num;
  int _ground_scan_index;
  float _sensor_mount_angle;

  Channel<ProjectionOut>& _output_channel;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_laser_cloud;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_info_cloud;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_ground_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud_pure;
  rclcpp::Publisher<cloud_msgs::msg::CloudInfo>::SharedPtr _pub_segmented_cloud_info;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloud;

  cloud_msgs::msg::CloudInfo _seg_msg;

  int _label_count;

  Eigen::MatrixXf _range_mat;   // range matrix for range image
  Eigen::MatrixXi _label_mat;   // label matrix for segmentaiton marking
  Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> _ground_mat;  // ground matrix for ground cloud marking

  std::vector<double> _Gk_star;
  std::vector<double> Gk_ref{0.0, 0.0, 1.0};

};



#endif  // IMAGEPROJECTION_H
