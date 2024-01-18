#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include <Eigen/QR>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include "lego_loam/tictoc.h"
#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <unistd.h>

class ImageProjection : public rclcpp::Node {
 public:

  ImageProjection(const std::string &name, Channel<ProjectionOut>& output_channel);

  ~ImageProjection() = default;

  void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
  void offlineKittiService(const geometry_msgs::msg::Twist::SharedPtr msg);

 private:
  void findStartEndAngle();
  void resetParameters();
  void projectPointCloud();
  void groundRemoval();
  void groundRemovalOurs();
  void cloudSegmentation();
  void labelComponents(int row, int col);
  void publishClouds();

  pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

  pcl::PointCloud<PointType>::Ptr _full_cloud;
  pcl::PointCloud<PointType>::Ptr _visual_cloud;
  pcl::PointCloud<PointType>::Ptr _full_info_cloud;

  pcl::PointCloud<PointType>::Ptr _ground_cloud;
  pcl::PointCloud<PointType>::Ptr _nonground_cloud;
  pcl::PointCloud<PointType>::Ptr _unknownground_cloud;
  pcl::PointCloud<PointType>::Ptr _nearground_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
  pcl::PointCloud<PointType>::Ptr _outlier_cloud;
  pcl::PointCloud<PointType>::Ptr _test_cloud;
  pcl::PointCloud<PointType>::Ptr pc_curr;

  std::string offline_path;
  int _vertical_scans;
  int _horizontal_scans;
  float _ang_bottom;
  float vertical_angle_top;
  float _ang_resolution_X;
  float _ang_resolution_Y;
  float _segment_theta;
  bool use_kitti;
  bool use_vlp32c;
  bool test = false;
  int _segment_valid_point_num;
  int _segment_valid_line_num;
  int _ground_scan_index;
  float _sensor_mount_angle;
  int lowerInd;
  float RVx;
  float RVy;
  float RVz;
  float D;
  float depth0;
  float depthPre;
  float height0;
  int frame_idx = 0;
  int frame_idx1 = 0;
  int rowIdn;
  double accumulated_run_time1 = 0;
  std::vector<int> vertical_raw;
  std::vector<int> vertical_raw2;
  std::vector<int> vertical_ori;
  std::vector<int> vertical_out;
  std::vector<int> vertical_cnt;

  // static double Timestart, Timeend;
  std::vector<double> gseg_runtime_list;
  std::vector<double> gseg_runtime_list1;
  std::vector<double> outlierCloudIntensity;
  std::vector<double> segmentedCloudIntensity;
  std::vector<double> filetime;

  Channel<ProjectionOut>& _output_channel;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_laser_cloud;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _sub_offline_kitti;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_info_cloud;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_ground_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_nonground_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_unknownground_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud_pure;
  rclcpp::Publisher<cloud_msgs::msg::CloudInfo>::SharedPtr _pub_segmented_cloud_info;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_visual_cloud;

  cloud_msgs::msg::CloudInfo _seg_msg;

  int _label_count;

  Eigen::MatrixXf _range_mat;   // range matrix for range image
  Eigen::MatrixXi _label_mat;   // label matrix for segmentaiton marking
  Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> _ground_mat;  // ground matrix for ground cloud marking


};



class KittiLoader {
public:
    KittiLoader(const std::string &abs_path) {
        pc_path_ = abs_path + "/velodyne";
        label_path_ = abs_path + "/labels";

        for (num_frames_ = 0;; num_frames_++) {
            std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % num_frames_).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }
        int num_labels;
        for (num_labels = 0;; num_labels++) {
            std::string filename = (boost::format("%s/%06d.label") % label_path_ % num_labels).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }

        if (num_frames_ == 0) {
            std::cerr << "\033[1;31mError: No files in " << pc_path_ << "\033[0m" << std::endl;
        }
        if (num_frames_ != num_labels) {
            std::cerr << "\033[1;31mError: The # of point clouds and # of labels are not same\033[0m" << std::endl;
        }
    }

    ~KittiLoader() {}

    size_t size() const { return num_frames_; }

    template<typename T>
    int get_cloud(size_t idx, pcl::PointCloud<T> &cloud) const {
        std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % idx).str();
        FILE *file = fopen(filename.c_str(), "rb");
        if (!file) {
            std::cerr << "error: failed to load " << filename << std::endl;
            return -1;
        }

        std::vector<float> buffer(1000000);
        size_t num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
        fclose(file);

        cloud.resize(num_points);
        if (std::is_same<T, pcl::PointXYZ>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
            }
        } else if (std::is_same<T, pcl::PointXYZI>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
                pt.intensity = buffer[i * 4 + 3];
            }

        } 
        // else if (std::is_same<T, PointXYZILID>::value) {
        //     std::string label_name = (boost::format("%s/%06d.label") % label_path_ % idx).str();
        //     std::ifstream label_input(label_name, std::ios::binary);
        //     if (!label_input.is_open()) {
        //         std::cerr << "Could not open the label!" << std::endl;
        //         return -1;
        //     }
        //     label_input.seekg(0, std::ios::beg);

        //     std::vector<uint32_t> labels(num_points);
        //     label_input.read((char*)&labels[0], num_points * sizeof(uint32_t));

        //     for (int i = 0; i < num_points; i++) {
        //         auto &pt = cloud.at(i);
        //         pt.x = buffer[i * 4];
        //         pt.y = buffer[i * 4 + 1];
        //         pt.z = buffer[i * 4 + 2];
        //         pt.intensity = buffer[i * 4 + 3];
        //         pt.label = labels[i] & 0xFFFF;
        //         pt.id = labels[i] >> 16;
        //     }

        // }
    }

private:
    int num_frames_;
    std::string label_path_;
    std::string pc_path_;
};



#endif  // IMAGEPROJECTION_H
