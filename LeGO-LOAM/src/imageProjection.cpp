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

#include <boost/circular_buffer.hpp>
#include "imageProjection.h"
using namespace std;

const std::string PARAM_VERTICAL_SCANS = "laser.num_vertical_scans";
const std::string PARAM_HORIZONTAL_SCANS = "laser.num_horizontal_scans";
const std::string PARAM_ANGLE_BOTTOM = "laser.vertical_angle_bottom";
const std::string PARAM_ANGLE_TOP = "laser.vertical_angle_top";
const std::string PARAM_GROUND_INDEX = "laser.ground_scan_index";
const std::string PARAM_SENSOR_ANGLE = "laser.sensor_mount_angle";
const std::string PARAM_USE_KITTI = "laser.use_kitti";
const std::string PARAM_USE_VLP32C = "laser.use_vlp32c";
const std::string PARAM_OFFLINE_PATH = "laser.offline_path";
const std::string PARAM_SEGMENT_THETA = "image_projection.segment_theta";
const std::string PARAM_SEGMENT_POINT = "image_projection.segment_valid_point_num";
const std::string PARAM_SEGMENT_LINE = "image_projection.segment_valid_line_num";

ImageProjection::ImageProjection(const std::string &name, Channel<ProjectionOut>& output_channel)
    : Node(name),  _output_channel(output_channel)
{
  _sub_laser_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_points", 1000, std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));
  _sub_offline_kitti = this->create_subscription<geometry_msgs::msg::Twist>(
    "/offline_kitti", 5, std::bind(&ImageProjection::offlineKittiService, this, std::placeholders::_1));

  _pub_full_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/full_cloud", 1);
  _pub_full_info_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/full_cloud_info", 1);
  _pub_ground_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_cloud", 1);
  _pub_nonground_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/nonground_cloud", 1);
  _pub_unknownground_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/unknownground_cloud", 1);
  _pub_segmented_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud", 1);
  _pub_segmented_cloud_pure = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud_pure", 1);
  _pub_segmented_cloud_info = this->create_publisher<cloud_msgs::msg::CloudInfo>("/segmented_cloud_info", 1);
  _pub_outlier_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/outlier_cloud", 1);
  // _pub_visual_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visual_cloud", 1);

  // Declare parameters
  this->declare_parameter(PARAM_VERTICAL_SCANS);
  this->declare_parameter(PARAM_HORIZONTAL_SCANS);
  this->declare_parameter(PARAM_ANGLE_BOTTOM);
  this->declare_parameter(PARAM_ANGLE_TOP);
  this->declare_parameter(PARAM_GROUND_INDEX);
  this->declare_parameter(PARAM_SENSOR_ANGLE);
  this->declare_parameter(PARAM_USE_KITTI);
  this->declare_parameter(PARAM_USE_VLP32C);
  this->declare_parameter(PARAM_OFFLINE_PATH);
  this->declare_parameter(PARAM_SEGMENT_THETA);
  this->declare_parameter(PARAM_SEGMENT_POINT);
  this->declare_parameter(PARAM_SEGMENT_LINE);

  // Read parameters
  if (!this->get_parameter(PARAM_VERTICAL_SCANS, _vertical_scans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_VERTICAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_HORIZONTAL_SCANS, _horizontal_scans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_HORIZONTAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_ANGLE_BOTTOM, _ang_bottom)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ANGLE_BOTTOM.c_str());
  }
  if (!this->get_parameter(PARAM_ANGLE_TOP, vertical_angle_top)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ANGLE_TOP.c_str());
  }
  if (!this->get_parameter(PARAM_GROUND_INDEX, _ground_scan_index)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s found", PARAM_GROUND_INDEX.c_str());
  }
  if (!this->get_parameter(PARAM_SENSOR_ANGLE, _sensor_mount_angle)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SENSOR_ANGLE.c_str());
  }
  if (!this->get_parameter(PARAM_USE_KITTI, use_kitti)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_USE_KITTI.c_str());
  }
  if (!this->get_parameter(PARAM_USE_VLP32C, use_vlp32c)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_USE_VLP32C.c_str());
  }
  if (!this->get_parameter(PARAM_OFFLINE_PATH, offline_path)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_OFFLINE_PATH.c_str());
  }
  if (!this->get_parameter(PARAM_SEGMENT_THETA, _segment_theta)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEGMENT_THETA.c_str());
  }
  if (!this->get_parameter(PARAM_SEGMENT_POINT, _segment_valid_point_num)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEGMENT_POINT.c_str());
  }
  if (!this->get_parameter(PARAM_SEGMENT_LINE, _segment_valid_line_num)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEGMENT_LINE.c_str());
  }

  _ang_resolution_X = (M_PI*2) / (_horizontal_scans);
  _ang_resolution_Y = DEG_TO_RAD*(vertical_angle_top - _ang_bottom) / float(_vertical_scans-1);
  _ang_bottom = -( _ang_bottom - 0.1) * DEG_TO_RAD;
  _segment_theta *= DEG_TO_RAD;
  _sensor_mount_angle *= DEG_TO_RAD;

  const size_t cloud_size = _vertical_scans * _horizontal_scans;

  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _visual_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _ground_cloud.reset(new pcl::PointCloud<PointType>());
  _nonground_cloud.reset(new pcl::PointCloud<PointType>());
  _unknownground_cloud.reset(new pcl::PointCloud<PointType>());
  _nearground_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());
  _test_cloud.reset(new pcl::PointCloud<PointType>());

  pc_curr.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
  _visual_cloud->points.resize(cloud_size);
  _full_info_cloud->points.resize(cloud_size);

}

void ImageProjection::resetParameters() {
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  PointType nanPoint;
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();

  _laser_cloud_in->clear();
  _ground_cloud->clear();
  _nonground_cloud->clear();
  _unknownground_cloud->clear();
  _nearground_cloud->clear();
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();
  _test_cloud->clear();
  outlierCloudIntensity.clear();
  segmentedCloudIntensity.clear();

  _range_mat.resize(_vertical_scans, _horizontal_scans);
  _ground_mat.resize(_vertical_scans, _horizontal_scans);
  _label_mat.resize(_vertical_scans, _horizontal_scans);

  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
  std::fill(_visual_cloud->points.begin(), _visual_cloud->points.end(), nanPoint);
  std::fill(_full_info_cloud->points.begin(), _full_info_cloud->points.end(),
            nanPoint);

  _seg_msg.start_ring_index.assign(_vertical_scans, 0);
  _seg_msg.end_ring_index.assign(_vertical_scans, 0);

  _seg_msg.segmented_cloud_ground_flag.assign(cloud_size, false);
  _seg_msg.segmented_cloud_col_ind.assign(cloud_size, 0);
  _seg_msg.segmented_cloud_range.assign(cloud_size, 0);
}

void ImageProjection::cloudHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
  frame_idx1++;
  // Reset parameters
  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(*laserCloudMsg, *_laser_cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);
  _seg_msg.header = laserCloudMsg->header;
  std::chrono::steady_clock::time_point time_start1 = std::chrono::steady_clock::now();
  findStartEndAngle();
  // Range image projection
  projectPointCloud();
  // Mark ground points
  groundRemovalOurs();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();

  // std::cout << "Frame Index IP:" << frame_idx1 << std::endl;
  double gseg_runtime_all1 = 0;
  std::chrono::steady_clock::time_point time_end1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds1 = time_end1 - time_start1;
  double runtime_frame1 = elapsed_seconds1.count() * 1000;
  gseg_runtime_list1.push_back(runtime_frame1/1000);
  for (size_t i = 0; i<gseg_runtime_list1.size(); ++i){
    gseg_runtime_all1 += gseg_runtime_list1[i];
  }
  float gseg_Hz1 = (gseg_runtime_all1/gseg_runtime_list1.size())*1000;
  // std::cout << "Image Projection Runtime (ms):" << gseg_Hz1 << std::endl;  //Alex debug
}

void ImageProjection::offlineKittiService(const geometry_msgs::msg::Twist::SharedPtr msg){
  KittiLoader loader((boost::format("%s/%s") % "/media/iec_lab/9696C55996C53A8F/KITTI/data_odometry_velodyne/dataset/sequences" % offline_path).str());
  int N = loader.size();
  // std::cout << "Frame Number:" << N << std::endl;  //Alex debug
  ifstream fin; 
  long double number;
  fin.open((boost::format("%s/%s/%s") % "/media/iec_lab/9696C55996C53A8F/KITTI/data_odometry_velodyne/dataset/sequences_calibration" % offline_path % "times.txt").str());
  while (fin >> number) {
    filetime.push_back(number);
  }
  fin.close();
  for (int n = 0; n < N; ++n) {
    pc_curr->clear();
    std::string filename = (boost::format("%s/%s/%s/%06d.bin") % "/media/iec_lab/9696C55996C53A8F/KITTI/data_odometry_velodyne/dataset/sequences" % offline_path % "velodyne" % n).str();
    FILE *file = fopen(filename.c_str(), "rb");
    std::vector<float> buffer(1000000);
    size_t num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);
    pc_curr->points.resize(num_points);
    for (int i = 0; i < num_points; i++) {
      pc_curr->points[i].x = buffer[i * 4];
      pc_curr->points[i].y = buffer[i * 4 + 1];
      pc_curr->points[i].z = buffer[i * 4 + 2];
      pc_curr->points[i].intensity = buffer[i * 4 + 3];
    }
    frame_idx1++;
    if((n % 100) == 0){
      // std::cout << "Frame Index:" << n << std::endl; //Alex debug
    }
    // Reset parameters
    resetParameters();

    // Copy and remove NAN points
    *_laser_cloud_in = *pc_curr;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);
    _seg_msg.header.stamp.sec = double(std::floor(filetime[n]));
    _seg_msg.header.stamp.nanosec = (filetime[n] - double(std::floor(filetime[n])))*1e9;
    _seg_msg.header.frame_id = "velo_link";
    std::chrono::steady_clock::time_point time_start1 = std::chrono::steady_clock::now();
    findStartEndAngle();
    // Range image projection
    projectPointCloud();
    // Mark ground points
    groundRemovalOurs();
    // Point cloud segmentation
    cloudSegmentation();
    //publish (optionally)
    publishClouds();

    // std::cout << "Frame Index IP:" << frame_idx1 << std::endl;
    double gseg_runtime_all1 = 0;
    std::chrono::steady_clock::time_point time_end1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = time_end1 - time_start1;
    double runtime_frame1 = elapsed_seconds1.count() * 1000;
    gseg_runtime_list1.push_back(runtime_frame1/1000);
    for (size_t i = 0; i<gseg_runtime_list1.size(); ++i){
      gseg_runtime_all1 += gseg_runtime_list1[i];
    }
    float gseg_Hz1 = (gseg_runtime_all1/gseg_runtime_list1.size())*1000;
    //  std::cout << "Image Projection Runtime (ms):" << gseg_Hz1 << std::endl; //Alex debug

    // while(!MOcomplete){
    //   // usleep(10000000);
    // }
    // MOcomplete = false;
    if(n==N-1){
      // std::cout << "Offline KITTI Complete" << std::endl;  //Alex debug
    }
    // if(n<=N-20){
      usleep(100000);
    // }else{
      // usleep(500000);
    // }
  }
}

void ImageProjection::projectPointCloud() {
  // range image projection
  const size_t cloudSize = _laser_cloud_in->points.size();
  if(!use_vlp32c){
    for (size_t i = 0; i < cloudSize; ++i) {
      PointType thisPoint = _laser_cloud_in->points[i];

      float range = sqrt(thisPoint.x * thisPoint.x +
                        thisPoint.y * thisPoint.y +
                        thisPoint.z * thisPoint.z);

      // find the row and column index in the image for this point
      float verticalAngle = std::asin(thisPoint.z / range);
          //std::atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y));

      int rowIdn = (verticalAngle + _ang_bottom) / _ang_resolution_Y;
      if (rowIdn < 0 || rowIdn >= _vertical_scans) {
        continue;
      }

      float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);

      int columnIdn = -round((horizonAngle - M_PI_2) / _ang_resolution_X) + _horizontal_scans * 0.5;

      if (columnIdn >= _horizontal_scans){
        columnIdn -= _horizontal_scans;
      }

      if (columnIdn < 0 || columnIdn >= _horizontal_scans){
        continue;
      }

      if (range < 0.1){
        continue;
      }

      _range_mat(rowIdn, columnIdn) = range;
      size_t index = columnIdn + rowIdn * _horizontal_scans;

      _visual_cloud->points[index] = thisPoint;

      thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

      _full_cloud->points[index] = thisPoint;
      // the corresponding range of a point is saved as "intensity"
      _full_info_cloud->points[index] = thisPoint;
      _full_info_cloud->points[index].intensity = range;
    }
  }else{
    vertical_ori.clear();
    vertical_raw.clear();
    for (size_t i = 0; i < cloudSize; ++i) {
      PointType thisPoint = _laser_cloud_in->points[i];
      float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
      float verticalAngle = std::asin(thisPoint.z / range);
      int temp = (verticalAngle + _ang_bottom) / (0.335*DEG_TO_RAD);
      vertical_ori.push_back(temp);
      if(temp<0){
        // vertical_out.push_back(temp);
        // _test_cloud->push_back(_laser_cloud_in->points[i]);
      }else{
        vertical_raw.push_back(temp);
      }
    }
    std::sort(vertical_raw.begin(), vertical_raw.end());
    vertical_raw2 = vertical_raw;
    auto uni_idx = std::unique(vertical_raw2.begin(), vertical_raw2.end());
    vertical_raw2.erase(uni_idx,vertical_raw2.end());
    // vertical_cnt = vertical_raw2;
    // for (size_t i=0; i<vertical_raw2.size(); ++i){
    //   vertical_cnt[i] = 0;
    // }
    // if(!test){
    //   for (size_t i=0; i<vertical_raw.size(); ++i){
    //     for (size_t j=0; j<vertical_raw2.size(); ++j){
    //       if (vertical_raw[i]==vertical_raw2[j]){
    //         vertical_cnt[j] = vertical_cnt[j] + 1;
    //         break;
    //       }
    //     }
    //   }
    //   for (size_t i=0; i<vertical_raw2.size(); ++i){
    //     std::cout << vertical_raw2[i] << ":" << vertical_cnt[i] << std::endl;
    //   }
    //   test = false;
    // }
    for (size_t i = 0; i < cloudSize; ++i) {
      PointType thisPoint = _laser_cloud_in->points[i];
      float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
      // find the row and column index in the image for this point
      float verticalAngle = std::asin(thisPoint.z / range);
      int temp = (verticalAngle + _ang_bottom) / (0.335*DEG_TO_RAD);
      if(temp<0){
        continue;
      }
      for (size_t j=0; j<vertical_raw2.size(); ++j){
        if (vertical_ori[i]==vertical_raw2[j]){
          rowIdn = int(j);
          break;
        }
      }
      float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);
      int columnIdn = -round((horizonAngle - M_PI_2) / _ang_resolution_X) + _horizontal_scans * 0.5;
      if (columnIdn >= _horizontal_scans){
        columnIdn -= _horizontal_scans;
      }
      if (columnIdn < 0 || columnIdn >= _horizontal_scans){
        continue;
      }
      if (range < 0.1){
        continue;
      }
      _range_mat(rowIdn, columnIdn) = range;
      size_t index = columnIdn + rowIdn * _horizontal_scans;

      _visual_cloud->points[index] = thisPoint;

      thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

      _full_cloud->points[index] = thisPoint;
      // the corresponding range of a point is saved as "intensity"
      _full_info_cloud->points[index] = thisPoint;
      _full_info_cloud->points[index].intensity = range;
    }
  }
}

void ImageProjection::findStartEndAngle() {
  // start and end orientation of this cloud
  auto point = _laser_cloud_in->points.front();
  _seg_msg.start_orientation = -std::atan2(point.y, point.x);

  point = _laser_cloud_in->points.back();
  _seg_msg.end_orientation = -std::atan2(point.y, point.x) + 2 * M_PI;

  if (_seg_msg.end_orientation - _seg_msg.start_orientation > 3 * M_PI) {
    _seg_msg.end_orientation -= 2 * M_PI;
  } else if (_seg_msg.end_orientation - _seg_msg.start_orientation < M_PI) {
    _seg_msg.end_orientation += 2 * M_PI;
  }
  _seg_msg.orientation_diff =
      _seg_msg.end_orientation - _seg_msg.start_orientation;
}

void ImageProjection::groundRemoval() {
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  // TicToc ground_segmentation_runtime;
  for (int j = 0; j < _horizontal_scans; ++j) {
    for (int i = 0; i < _ground_scan_index; ++i) {
      int lowerInd = j + (i)*_horizontal_scans;
      int upperInd = j + (i + 1) * _horizontal_scans;

      if (_full_cloud->points[lowerInd].intensity == -1 ||
          _full_cloud->points[upperInd].intensity == -1) {
        // no info to check, invalid points
        _ground_mat(i, j) = -1;
        continue;
      }

      float dX =
          _full_cloud->points[upperInd].x - _full_cloud->points[lowerInd].x;
      float dY =
          _full_cloud->points[upperInd].y - _full_cloud->points[lowerInd].y;
      float dZ =
          _full_cloud->points[upperInd].z - _full_cloud->points[lowerInd].z;

      float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY + dZ * dZ));

      // TODO: review this change

      if ( (vertical_angle - _sensor_mount_angle) <= 10 * DEG_TO_RAD) {
        _ground_mat(i, j) = 1;
        _ground_mat(i + 1, j) = 1;
      }
    }
  }
  // double runtime_frame;
  // double gseg_runtime_all = 0;
  // ground_segmentation_runtime.toc("Ground Segmentation Runtime", runtime_frame);
  // runtime_frame = ground_segmentation_runtime.toc("Ground Segmentation Runtime");
  // gseg_runtime_list.push_back(runtime_frame/1000);
  // // gseg_runtime_list[frame_idx] = runtime_frame/1000;
  // frame_idx++;
  // for (int i = 0; i<frame_idx; ++i){
  //   gseg_runtime_all += gseg_runtime_list[i];
  // }
  // float gseg_Hz = 1/(gseg_runtime_all/frame_idx);
  // std::cout << "Frame Index:" << frame_idx << std::endl;
  // std::cout << "Ground Segmentation Runtime (Hz):" << gseg_Hz << std::endl;

  // extract ground cloud (_ground_mat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~_N_scan-1, need _range_mat
  // for mark label matrix for the 16th scan
  for (int i = 0; i < _vertical_scans; ++i) {
    for (int j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1 ||
          _range_mat(i, j) == FLT_MAX) {
        _label_mat(i, j) = -1;
      }
    }
  }

  for (int i = 0; i < _vertical_scans; ++i) {
    for (int j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1){
        _ground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
      }else{
        _nonground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
      }
    }
  }
  // std::cout << "Ground Cloud Number:" << _ground_cloud->points.size() << std::endl;
  // std::cout << "Nonground Cloud Number:" << _nonground_cloud->points.size() << std::endl;
}

void ImageProjection::groundRemovalOurs() {
  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
  for (int j = 0; j < _horizontal_scans; ++j) {
    // int NEAR_num_all = 0;
    // int NEAR_num_nong = 0;
    // float NEAR_thedaPre = 0;
    bool findIRV = false;
    bool findFilterObs = false;
    for (int i = 0; i < _vertical_scans; ++i) {
      int validInd = j + (i)*_horizontal_scans;
      // Debug
      // std::cout << "j0:" << _full_cloud->points[validInd].x << "_" << _full_cloud->points[validInd].y << "_" << _full_cloud->points[validInd].z << "_" << _full_cloud->points[validInd].intensity << std::endl;
      // Nan Point Check
      if (_full_cloud->points[validInd].intensity == 0){
        _ground_mat(i,j) = -1;
        continue;
      }
      
      // % MAIN
      if (!findIRV){
        float depth0 = sqrt(_full_cloud->points[validInd].x * _full_cloud->points[validInd].x + _full_cloud->points[validInd].y * _full_cloud->points[validInd].y);
        // float depthPre = depth0;
        // float height0 = _full_cloud->points[validInd].z;
        // Initial Reference Vector
        RVx = _full_cloud->points[validInd].x/depth0;
        RVy = _full_cloud->points[validInd].y/depth0;
        RVz = 0;
        findIRV = true;
        lowerInd = validInd;
        _ground_mat(i,j) = 1;
      }else{
        int upperInd = validInd;
        // Target Vector
        float TVx = _full_cloud->points[upperInd].x - _full_cloud->points[lowerInd].x;
        float TVy = _full_cloud->points[upperInd].y - _full_cloud->points[lowerInd].y;
        float TVz = _full_cloud->points[upperInd].z - _full_cloud->points[lowerInd].z;
        // Vector Iteration and Classification
        float angleMain = std::acos((TVx*RVx+TVy*RVy+TVz*RVz)/(sqrt(TVx*TVx+TVy*TVy+TVz*TVz)*sqrt(RVx*RVx+RVy*RVy+RVz*RVz)))/DEG_TO_RAD;
        _ground_mat(i,j) = 1;
        if (use_kitti){
          if (i<16){
            D = 60;
          }else{
            D = 25;
          }
        }else if (use_vlp32c){
          D = 25;
        }else{
          D = 12.5;
        }
        if (angleMain <= D){
          RVx += TVx;
          RVy += TVy;
          RVz += TVz;
          _ground_mat(i,j) = 1;
        }else{
          _ground_mat(i,j) = 0;
        }
        lowerInd = upperInd;
      }
      // // % NEAR
      // float depth = sqrt(_full_cloud->points[validInd].x * _full_cloud->points[validInd].x + _full_cloud->points[validInd].y * _full_cloud->points[validInd].y);
      // if (depth>5.5){
      //   continue;
      // }
      // NEAR_num_all++;
      // _ground_mat(i,j) = 1;
      // float NEAR_dz = _full_cloud->points[validInd].z - height0;
      // float NEAR_dr = depth - depthPre;
      // float NEAR_theda = std::acos((depth0*depth)/(depth0*sqrt(depth*depth+NEAR_dz*NEAR_dz)))/DEG_TO_RAD;
      // float NEAR_dtheda = NEAR_theda - NEAR_thedaPre;
      // if (NEAR_dr<=0.04 && NEAR_dtheda>=0.6){
      //   _ground_mat(i,j) = 0;
      //   NEAR_num_nong++;
      // }
      // if (NEAR_num_all<3){
      //   _ground_mat(i,j) = 1;
      // }
      // depthPre = depth;
      // NEAR_thedaPre = NEAR_theda;
    }
    // if (NEAR_num_all>0 && (NEAR_num_nong/NEAR_num_all)>=0.2){
    //   bool findObs = false;
    //   for (int i = 0; i < NEAR_num_all; ++i) {
    //     if(_ground_mat(i,j)==1 && !findObs){
    //       continue;
    //     }
    //     findObs = true;
    //     _ground_mat(i,j) = 0;
    //   }
    // }
    // for (int i = 0; i < NEAR_num_all; ++i) {
    //   int nearInd = j + (i)*_horizontal_scans;
    //   if (_full_cloud->points[nearInd].z>=-1.3){
    //     _ground_mat(i,j) = 0;
    //   }
    // }
    // % Filter
    for (int i = 0; i < _vertical_scans; ++i) {
      if (_ground_mat(i,j) != 0 && !findFilterObs){
        continue;
      }
      findFilterObs = true;
      if (_ground_mat(i,j) == 1){
        _ground_mat(i,j) = 2;
      }
    }
  }

  // % ADD
  for (int i = 0; i < _vertical_scans; ++i) {
    for (int j = 2; j < _horizontal_scans; ++j) {
      if(_ground_mat(i,j)!=2){
        continue;
      }
      bool ADD_neighbor = (_ground_mat(i,j-2) == 1 || _ground_mat(i,j-1) == 1);
      int ADD_Ind = j + (i)*_horizontal_scans;
      int ADD_nei_Ind = j - 2 + (i)*_horizontal_scans;
      float ADD_x0 = _full_cloud->points[ADD_Ind].x;
      float ADD_y0 = _full_cloud->points[ADD_Ind].y;
      float ADD_z0 = _full_cloud->points[ADD_Ind].z;
      float ADD_r = sqrt(ADD_x0*ADD_x0 + ADD_y0*ADD_y0 + ADD_z0*ADD_z0);
      float ADD_dx = _full_cloud->points[ADD_Ind].x - _full_cloud->points[ADD_nei_Ind].x;
      float ADD_dy = _full_cloud->points[ADD_Ind].y - _full_cloud->points[ADD_nei_Ind].y;
      float ADD_dz = _full_cloud->points[ADD_Ind].z - _full_cloud->points[ADD_nei_Ind].z;
      float ADD_dr = sqrt(ADD_dx*ADD_dx + ADD_dy*ADD_dy + ADD_dz*ADD_dz);
      if (ADD_neighbor && ADD_dr <= 0.061*ADD_r && ADD_dz <= 0.1){
        _ground_mat(i,j) = 1;
      }
    }
    for (int j = _horizontal_scans - 3; j > -1; --j) {
      if(_ground_mat(i,j)!=2){
        continue;
      }
      bool ADD_neighbor = (_ground_mat(i,j+2) == 1 || _ground_mat(i,j+1) == 1);
      int ADD_Ind = j + (i)*_horizontal_scans;
      int ADD_nei_Ind = j + 2 + (i)*_horizontal_scans;
      float ADD_x0 = _full_cloud->points[ADD_Ind].x;
      float ADD_y0 = _full_cloud->points[ADD_Ind].y;
      float ADD_z0 = _full_cloud->points[ADD_Ind].z;
      float ADD_r = sqrt(ADD_x0*ADD_x0 + ADD_y0*ADD_y0 + ADD_z0*ADD_z0);
      float ADD_dx = _full_cloud->points[ADD_Ind].x - _full_cloud->points[ADD_nei_Ind].x;
      float ADD_dy = _full_cloud->points[ADD_Ind].y - _full_cloud->points[ADD_nei_Ind].y;
      float ADD_dz = _full_cloud->points[ADD_Ind].z - _full_cloud->points[ADD_nei_Ind].z;
      float ADD_dr = sqrt(ADD_dx*ADD_dx + ADD_dy*ADD_dy + ADD_dz*ADD_dz);
      if (ADD_neighbor && ADD_dr <= 0.061*ADD_r && ADD_dz <= 0.1){
        _ground_mat(i,j) = 1;
      }
    }
  }

  // % ELEVATION
  float ELE_H_temp = -1.3;
  float ELE_H = -1.3;
  for (int j = 0; j < _horizontal_scans; ++j) {
    int ELE_gnum = 0;
    for (int i = 0; i < _vertical_scans; ++i) {
      if (_ground_mat(i,j) == 1){
        int ELE_Ind = j + (i)*_horizontal_scans;
        ELE_gnum++;
        ELE_H_temp = _full_cloud->points[ELE_Ind].z;
      }
    }
    if (ELE_gnum>=5){
      ELE_H = ELE_H_temp;
    }
    for (int i = 0; i < _vertical_scans; ++i) {
      if (_ground_mat(i,j) == 2){
        int ELE_Ind = j + (i)*_horizontal_scans;
        if (_full_cloud->points[ELE_Ind].z < ELE_H+0.3){
          _ground_mat(i,j) = 1;
        }else{
          _ground_mat(i,j) = 0;
        }
      }
    }
  }

  // % NEAR
  int NEAR_num = 0;
  for (int i = 0; i < _vertical_scans; ++i) {
    for (int j = 0; j < _horizontal_scans; ++j) {
      int NearInd = j + (i)*_horizontal_scans;
      float depth = sqrt(_full_cloud->points[NearInd].x * _full_cloud->points[NearInd].x + _full_cloud->points[NearInd].y * _full_cloud->points[NearInd].y);
      if (depth<=10 && _ground_mat(i,j) == 1){
        _nearground_cloud->push_back(_full_cloud->points[NearInd]);
        _nearground_cloud->points[NEAR_num].intensity = NearInd;
        NEAR_num++;
        if (depth<=5){
          _ground_mat(i,j) = 0;
        }
      }
    }
  }
  pcl::SampleConsensusModelPlane<PointType>::Ptr model_p (new pcl::SampleConsensusModelPlane<PointType> (_nearground_cloud));
  std::vector<int> inliers;
  pcl::RandomSampleConsensus<PointType> ransac (model_p);
  ransac.setDistanceThreshold (0.5);
  ransac.computeModel();
  ransac.getInliers(inliers);
  // Debug
  // for (size_t i = 0; i < inliers.size(); ++i) {
  //       std::cout << inliers[i] << "; ";
  // }
  // std::cout << std::endl;
  for (size_t k = 0; k < inliers.size(); ++k) {
    int NearInd = _nearground_cloud->points[inliers[k]].intensity;
    float depth = sqrt(_full_cloud->points[NearInd].x * _full_cloud->points[NearInd].x + _full_cloud->points[NearInd].y * _full_cloud->points[NearInd].y);
    if (depth<=5){
      int i = NearInd / _horizontal_scans;
      int j = NearInd - (i * _horizontal_scans);
      _ground_mat(i,j) = 1;
    }
  }

  // Runtime
  double gseg_runtime_all = 0;
  std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = time_end - time_start;
  double runtime_frame = elapsed_seconds.count() * 1000;
  gseg_runtime_list.push_back(runtime_frame/1000);
  frame_idx++;
  for (size_t i = 0; i<gseg_runtime_list.size(); ++i){
    gseg_runtime_all += gseg_runtime_list[i];
  }
  float gseg_Hz = 1/(gseg_runtime_all/gseg_runtime_list.size());
  // std::cout << "Frame Index:" << frame_idx << std::endl;
  // std::cout << "Ground Segmentation Runtime (Hz):" << gseg_Hz << std::endl;

  // Push Back
  for (int i = 0; i < _vertical_scans; ++i) {
    for (int j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1 ||
          _range_mat(i, j) == FLT_MAX) {
        _label_mat(i, j) = -1;
      }
    }
  }
  for (int i = 0; i < _vertical_scans; ++i) {
    for (int j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1){
        _ground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
      }else if (_ground_mat(i, j) == 0){
        _nonground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
      }else if (_ground_mat(i, j) == 2){
        _unknownground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
      }
    }
  }
  // std::cout << "Ground Cloud Number:" << _ground_cloud->points.size() << std::endl;
  // std::cout << "Nonground Cloud Number:" << _nonground_cloud->points.size() << std::endl;
  // std::cout << "Unknownground Cloud Number:" << _unknownground_cloud->points.size() << std::endl;
}

void ImageProjection::cloudSegmentation() {
  // segmentation process
  for (int i = 0; i < _vertical_scans; ++i)
    for (int j = 0; j < _horizontal_scans; ++j)
      if (_label_mat(i, j) == 0) labelComponents(i, j);

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (int i = 0; i < _vertical_scans; ++i) {
    _seg_msg.start_ring_index[i] = sizeOfSegCloud - 1 + 5;

    for (int j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 || _ground_mat(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (_label_mat(i, j) == 999999) {
          if (i > _ground_scan_index && j % 5 == 0) {
            _outlier_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
            outlierCloudIntensity.push_back(_visual_cloud->points[j + i * _horizontal_scans].intensity);
            // _outlier_cloud->push_back(_visual_cloud->points[j + i * _horizontal_scans]);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        if (_ground_mat(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < _horizontal_scans - 5) continue;
        }
        // mark ground points so they will not be considered as edge features
        // later
        _seg_msg.segmented_cloud_ground_flag[sizeOfSegCloud] =
            (_ground_mat(i, j) == 1);
        // mark the points' column index for marking occlusion later
        _seg_msg.segmented_cloud_col_ind[sizeOfSegCloud] = j;
        // save range info
        _seg_msg.segmented_cloud_range[sizeOfSegCloud] =
            _range_mat(i, j);
        // save seg cloud
        _segmented_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
        // std::cout << "_segmented_cloud Form : " << _full_cloud->points[j + i * _horizontal_scans] << std::endl; // Alex
        segmentedCloudIntensity.push_back(_visual_cloud->points[j + i * _horizontal_scans].intensity);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }

    _seg_msg.end_ring_index[i] = sizeOfSegCloud - 1 - 5;
  }

  // extract segmented cloud for visualization
  for (int i = 0; i < _vertical_scans; ++i) {
    for (int j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 && _label_mat(i, j) != 999999) {
        _segmented_cloud_pure->push_back(
            _full_cloud->points[j + i * _horizontal_scans]);
        _segmented_cloud_pure->points.back().intensity =
            _label_mat(i, j);
      }
    }
  }
}

void ImageProjection::labelComponents(int row, int col) {

  const float segmentThetaThreshold = tan(_segment_theta);

  std::vector<bool> lineCountFlag(_vertical_scans, false);
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  using Coord2D = Eigen::Vector2i;
  boost::circular_buffer<Coord2D> queue(cloud_size);
  boost::circular_buffer<Coord2D> all_pushed(cloud_size);

  queue.push_back({ row,col } );
  all_pushed.push_back({ row,col } );

  const Coord2D neighborIterator[4] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (queue.size() > 0) {
    // Pop point
    Coord2D fromInd = queue.front();
    queue.pop_front();

    // Mark popped point
    _label_mat(fromInd.x(), fromInd.y()) = _label_count;
    // Loop through all the neighboring grids of popped grid

    for (const auto& iter : neighborIterator) {
      // new index
      int thisIndX = fromInd.x() + iter.x();
      int thisIndY = fromInd.y() + iter.y();
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= _vertical_scans){
        continue;
      }
      // at range image margin (left or right side)
      if (thisIndY < 0){
        thisIndY = _horizontal_scans - 1;
      }
      if (thisIndY >= _horizontal_scans){
        thisIndY = 0;
      }
      // prevent infinite loop (caused by put already examined point back)
      if (_label_mat(thisIndX, thisIndY) != 0){
        continue;
      }

      float d1 = std::max(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));
      float d2 = std::min(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));

      float alpha = (iter.x() == 0) ? _ang_resolution_X : _ang_resolution_Y;
      float tang = (d2 * sin(alpha) / (d1 - d2 * cos(alpha)));

      if (tang > segmentThetaThreshold) {
        queue.push_back( {thisIndX, thisIndY } );

        _label_mat(thisIndX, thisIndY) = _label_count;
        lineCountFlag[thisIndX] = true;

        all_pushed.push_back(  {thisIndX, thisIndY } );
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  if (all_pushed.size() >= 30){
    feasibleSegment = true;
  }
  else if (static_cast<int>(all_pushed.size()) >= _segment_valid_point_num) {
    int lineCount = 0;
    for (int i = 0; i < _vertical_scans; ++i) {
      if (lineCountFlag[i] == true) ++lineCount;
    }
    if (lineCount >= _segment_valid_line_num) feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true) {
    ++_label_count;
  } else {  // segment is invalid, mark these points
    for (size_t i = 0; i < all_pushed.size(); ++i) {
      _label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
    }
  }
}

void ImageProjection::publishClouds() {


  // int cloudSize = _visual_cloud->points.size();
  // PointType point;
  // for (int i = 0; i < cloudSize; i++) {
  //   point.x = _visual_cloud->points[i].y;
  //   point.y = _visual_cloud->points[i].z;
  //   point.z = _visual_cloud->points[i].x;
  //   point.intensity =_visual_cloud->points[i].intensity;
  //   _visual_cloud->points[i] = point;
  // }


  sensor_msgs::msg::PointCloud2 temp;

  auto PublishCloud = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (pub->get_subscription_count() != 0) {
      pcl::toROSMsg(*cloud, temp);
      temp.header.stamp = _seg_msg.header.stamp;
      temp.header.frame_id = "base_link";
      pub->publish(temp);
    }
  };

  auto PublishCloudCamera = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (pub->get_subscription_count() != 0) {
      pcl::toROSMsg(*cloud, temp);
      temp.header.stamp = _seg_msg.header.stamp;
      temp.header.frame_id = "camera";
      pub->publish(temp);
    }
  };

  PublishCloud(_pub_outlier_cloud, _outlier_cloud);
  PublishCloud(_pub_segmented_cloud, _segmented_cloud);
  PublishCloud(_pub_full_cloud, _full_cloud);
  PublishCloud(_pub_ground_cloud, _ground_cloud);
  PublishCloud(_pub_nonground_cloud, _nonground_cloud);
  PublishCloud(_pub_unknownground_cloud, _unknownground_cloud);
  PublishCloud(_pub_segmented_cloud_pure, _segmented_cloud_pure);
  PublishCloud(_pub_full_info_cloud, _full_info_cloud);
  // PublishCloudCamera(_pub_visual_cloud, _visual_cloud);

  if (_pub_segmented_cloud_info->get_subscription_count() != 0) {
    _pub_segmented_cloud_info->publish(_seg_msg);
  }

  //--------------------
  ProjectionOut out;
  out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  out.segmented_cloud.reset(new pcl::PointCloud<PointType>());
  // out.visual_cloud.reset(new pcl::PointCloud<PointType>());
  // out.full_cloud.reset(new pcl::PointCloud<PointType>());

  std::swap( out.seg_msg, _seg_msg);
  std::swap(out.outlier_cloud, _outlier_cloud);
  std::swap(out.segmented_cloud, _segmented_cloud);
  // std::swap(out.visual_cloud, _visual_cloud);
  // std::swap(out.full_cloud, _full_cloud);
  out.outlierCloud_Intensity = outlierCloudIntensity;
  out.segmentedCloud_Intensity = segmentedCloudIntensity;

  _output_channel.send( std::move(out) );

}