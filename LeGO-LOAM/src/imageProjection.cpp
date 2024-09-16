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

// Alex
#include <cstdlib>  // std::div
#include <memory>
#include <iostream>
#include <limits>
#include <cmath>
#include <vector>



const std::string PARAM_VERTICAL_SCANS = "laser.num_vertical_scans";
const std::string PARAM_HORIZONTAL_SCANS = "laser.num_horizontal_scans";
const std::string PARAM_ANGLE_BOTTOM = "laser.vertical_angle_bottom";
const std::string PARAM_ANGLE_TOP = "laser.vertical_angle_top";
const std::string PARAM_GROUND_INDEX = "laser.ground_scan_index";
const std::string PARAM_SENSOR_ANGLE = "laser.sensor_mount_angle";
const std::string PARAM_SEGMENT_THETA = "image_projection.segment_theta";
const std::string PARAM_SEGMENT_POINT = "image_projection.segment_valid_point_num";
const std::string PARAM_SEGMENT_LINE = "image_projection.segment_valid_line_num";

ImageProjection::ImageProjection(const std::string &name, Channel<ProjectionOut>& output_channel)
    : Node(name),  _output_channel(output_channel)
{
  _sub_laser_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_points", 1, std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));

  _pub_full_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/full_cloud_projected", 1);
  _pub_full_info_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/full_cloud_info", 1);
  _pub_ground_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_cloud", 1);
  _pub_segmented_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud", 1);
  _pub_segmented_cloud_pure = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud_pure", 1);
  _pub_segmented_cloud_info = this->create_publisher<cloud_msgs::msg::CloudInfo>("/segmented_cloud_info", 1);
  _pub_outlier_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/outlier_cloud", 1);

  // Declare parameters
  this->declare_parameter(PARAM_VERTICAL_SCANS);
  this->declare_parameter(PARAM_HORIZONTAL_SCANS);
  this->declare_parameter(PARAM_ANGLE_BOTTOM);
  this->declare_parameter(PARAM_ANGLE_TOP);
  this->declare_parameter(PARAM_GROUND_INDEX);
  this->declare_parameter(PARAM_SENSOR_ANGLE);
  this->declare_parameter(PARAM_SEGMENT_THETA);
  this->declare_parameter(PARAM_SEGMENT_POINT);
  this->declare_parameter(PARAM_SEGMENT_LINE);

  float vertical_angle_top;

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
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _ground_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
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
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();

  _range_mat.resize(_vertical_scans, _horizontal_scans);
  _ground_mat.resize(_vertical_scans, _horizontal_scans);
  _label_mat.resize(_vertical_scans, _horizontal_scans);

  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
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
  // Reset parameters
  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(*laserCloudMsg, *_laser_cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);
  _seg_msg.header = laserCloudMsg->header;

  findStartEndAngle();
  // Range image projection
  projectPointCloud();
  // Mark ground points
  groundRemoval();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();
}


void ImageProjection::projectPointCloud() {
  // range image projection
  const size_t cloudSize = _laser_cloud_in->points.size();

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

    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    size_t index = columnIdn + rowIdn * _horizontal_scans;
    _full_cloud->points[index] = thisPoint;
    // the corresponding range of a point is saved as "intensity"
    _full_info_cloud->points[index] = thisPoint;
    _full_info_cloud->points[index].intensity = range;
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

  /////////////////////////////////////////////////////////////////////////////////////
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

      // float dX =
      //     _full_cloud->points[upperInd].x - _full_cloud->points[lowerInd].x;
      // float dY =
      //     _full_cloud->points[upperInd].y - _full_cloud->points[lowerInd].y;
      // float dZ =
      //     _full_cloud->points[upperInd].z - _full_cloud->points[lowerInd].z;

      // float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY + dZ * dZ));

      // // TODO: review this change

      // if ( (vertical_angle - _sensor_mount_angle) <= 10 * DEG_TO_RAD) {
      //   _ground_mat(i, j) = 1;
      //   _ground_mat(i + 1, j) = 1;
      // }
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////////////
  // Alex ransac extract ground plane
  // 從剛才提取的點雲中轉換成我們的自定義 Point 結構並進行濾波
  std::vector<Point> custom_cloud;
  int index = 0;  // 用來追蹤點的索引
  for (const auto& pcl_point : _full_cloud->points) {
      // std::cout << "z value : " << pcl_point.z << std::endl;
      if (pcl_point.z >= -0.05-0.035-0.05 && pcl_point.z <= 0.05-0.035-0.05) { //pcl_point.z >= -0.05-0.035-0.05 && pcl_point.z <= 0.05-0.035-0.05
          Point pt = {pcl_point.x, pcl_point.y, pcl_point.z, index};
          custom_cloud.push_back(pt);
      }
      index++;
  }
  // 設置點雲的 width 和 height 屬性
  _full_cloud->width = _full_cloud->points.size();  // 點的數量
  _full_cloud->height = 1;  // 非結構化點雲
  _full_cloud->is_dense = false;  // 如果點雲可能包含無效點（例如 NaN）
  // 儲存為 .pcd 檔案
  pcl::io::savePCDFileASCII("full_cloud.pcd", *_full_cloud);
  
  // 將 custom_cloud 轉換為 PCL 格式
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> filteredclouds_idx;
  for (const auto& pt : custom_cloud) {
      pcl::PointXYZ pcl_point;
      pcl_point.x = pt.x;
      pcl_point.y = pt.y;
      pcl_point.z = pt.z;
      filtered_cloud->points.push_back(pcl_point);

      filteredclouds_idx.push_back(pt.index);
  }
  // 設置點雲的 width 和 height 屬性
  filtered_cloud->width = filtered_cloud->points.size();  // 點的數量
  filtered_cloud->height = 1;  // 非結構化點雲
  filtered_cloud->is_dense = false;  // 如果點雲可能包含無效點（例如 NaN）
  // 儲存為 .pcd 檔案
  pcl::io::savePCDFileASCII("filtered_pointcloud.pcd", *filtered_cloud);
  // std::cout << "Saved filtered point cloud to filtered_pointcloud.pcd" << std::endl;
  

  // convert to <GRANSAC::AbstractParameter>
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Qk = ConvertPointCloudToGRANSAC(filtered_cloud, filteredclouds_idx);

  // caculate RANSAC
  GRANSAC::RANSAC<PlaneModel, 3> Estimator;
  Estimator.Initialize(0.1, 100); // Threshold, iterations
  int64_t start = cv::getTickCount();
	Estimator.Estimate(Qk);
	int64_t end = cv::getTickCount();
  // std::cout << "RANSAC took: " << GRANSAC::VPFloat(end - start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms." << std::endl;

  auto Ak = Estimator.GetBestInliers();

  // 將 custom_cloud 轉換為 PCL 格式
  pcl::PointCloud<pcl::PointXYZ>::Ptr Ground_Plane(new pcl::PointCloud<pcl::PointXYZ>);
  
  for (const auto& pt : Ak) {
      auto point = std::dynamic_pointer_cast<Point3D>(pt);
      if (point) {
          pcl::PointXYZ pcl_point;
          pcl_point.x = point->m_Point3D[0];
          pcl_point.y = point->m_Point3D[1];
          pcl_point.z = point->m_Point3D[2];
          Ground_Plane->points.push_back(pcl_point);
          int index = point->m_Index;
          
          // 使用 std::div 函数计算商和余数
          div_t result = std::div(index, _horizontal_scans);
          int i = result.quot;int j = result.rem;

          _ground_mat(i, j) = 1;
      }
  }

  // 設置點雲的 width 和 height 屬性
  Ground_Plane->width = Ground_Plane->points.size();  // 點的數量
  Ground_Plane->height = 1;  // 非結構化點雲
  Ground_Plane->is_dense = false;  // 如果點雲可能包含無效點（例如 NaN）
  // 儲存為 .pcd 檔案
  pcl::io::savePCDFileASCII("Ground_Plane.pcd", *Ground_Plane);

  auto BestPlane = Estimator.GetBestModel();
  std::vector<double> Gk(4);
  for (int i = 0; i < 4; i++)
  {
    Gk[i] = BestPlane->m_PlaneCoefs[i];
  }

  std::vector<double> original = {0.0, 0.0, 0.0};
  double dk_star = calculateDistance(original, Gk);

  
  _Gk_star.resize(4);
  std::vector<double> vec;
  // 將 Gk 的前三個值複製到 _Gk_star
  for (int i = 0; i < 3; i++)
  {
    _Gk_star[i] = Gk[i];
  }
  vec.assign(_Gk_star.begin(), _Gk_star.begin() + 3);
  double angle = dotProduct(Gk_ref, vec);
  if (angle < 0.0){
    for (auto& element : _Gk_star) {
        element = -element;
    }
  }
  
  // 將 dk_star 填入 _Gk_star 的最後一個位置
  _Gk_star[3] = dk_star;
  // std::cout << "Ground Plane Coefficient = ";
  // for (const auto& value : _Gk_star) {
  //     std::cout << value << " ";
  // }
  // std::cout << std::endl;
  /////////////////////////////////////////////////////////////////////////////////////


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

  for (int i = 0; i <= _ground_scan_index; ++i) {
    for (int j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1)
        _ground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
    }
  }
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
            _outlier_cloud->push_back(
                _full_cloud->points[j + i * _horizontal_scans]);
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

  sensor_msgs::msg::PointCloud2 temp;
  temp.header.stamp = _seg_msg.header.stamp;
  temp.header.frame_id = "base_link";

  auto PublishCloud = [](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, sensor_msgs::msg::PointCloud2& temp,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (pub->get_subscription_count() != 0) {
      pcl::toROSMsg(*cloud, temp);
      pub->publish(temp);
    }
  };

  PublishCloud(_pub_outlier_cloud, temp, _outlier_cloud);
  PublishCloud(_pub_segmented_cloud, temp, _segmented_cloud);
  PublishCloud(_pub_full_cloud, temp, _full_cloud);
  PublishCloud(_pub_ground_cloud, temp, _ground_cloud);
  PublishCloud(_pub_segmented_cloud_pure, temp, _segmented_cloud_pure);
  PublishCloud(_pub_full_info_cloud, temp, _full_info_cloud);

  if (_pub_segmented_cloud_info->get_subscription_count() != 0) {
    _pub_segmented_cloud_info->publish(_seg_msg);
  }

  //--------------------
  ProjectionOut out;
  out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  out.segmented_cloud.reset(new pcl::PointCloud<PointType>());

  std::swap( out.seg_msg, _seg_msg);
  std::swap(out.outlier_cloud, _outlier_cloud);
  std::swap(out.segmented_cloud, _segmented_cloud);
  std::swap(out.Gk_star, _Gk_star);

  _output_channel.send( std::move(out) );

}


// Alex
std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> ImageProjection::ConvertPointCloudToGRANSAC(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const std::vector<int>& original_indices)
{
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    CandPoints.resize(cloud->points.size());

    // 如果沒有提供 original_indices，則自行生成索引
    bool use_provided_indices = !original_indices.empty();

#pragma omp parallel for num_threads(6)
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const pcl::PointXYZ& p = cloud->points[i];
        int index = use_provided_indices ? original_indices[i] : i; // 如果提供了索引，則使用原始索引，否則使用當前的索引
        std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point3D>(p.x, p.y, p.z, index);
        CandPoints[i] = CandPt;
    }

    return CandPoints;
}

// 函數用來計算點到平面的距離
double ImageProjection::calculateDistance(const std::vector<double>& p, const std::vector<double>& Gk) {
    return std::abs(Gk[0]*p[0] + Gk[1]*p[1] + Gk[2]*p[2] + Gk[3]) / 
           std::sqrt(Gk[0]*Gk[0] + Gk[1]*Gk[1] + Gk[2]*Gk[2]);
}

double ImageProjection::dotProduct(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    if (vec1.size() != vec2.size()) {
        throw std::invalid_argument("兩個向量的大小不同，無法計算內積");
    }

    double result = 0.0;
    for (size_t i = 0; i < vec1.size(); ++i) {
        result += vec1[i] * vec2[i];  // 逐個相乘並相加
    }
    return result;
}