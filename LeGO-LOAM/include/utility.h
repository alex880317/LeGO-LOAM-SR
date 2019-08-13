#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

typedef pcl::PointXYZI  PointType;

typedef Eigen::Vector3f Vector3;

static const std::string pointCloudTopic = "/velodyne_points";
static const std::string imuTopic = "/imu/data";

// VLP-16
static const int N_SCAN = 16;
static const int HORIZONTAL_SCAN = 1800;
static const float ang_res_x = 0.2;
static const float ang_res_y = 2.0;
static const float ang_bottom = 15.0+0.1;
static const int groundScanInd = 7;

// HDL-32E
// static const int N_SCAN = 32;
// static const int HORIZONTAL_SCAN = 1800;
// static const float ang_res_x = 360.0/float(HORIZONTAL_SCAN);
// static const float ang_res_y = 41.33/float(N_SCAN-1);
// static const float ang_bottom = 30.67;
// static const int groundScanInd = 20;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not supported yet, please just publish point cloud data
// Ouster OS1-16
// static const int N_SCAN = 16;
// static const int HORIZONTAL_SCAN = 1024;
// static const float ang_res_x = 360.0/float(HORIZONTAL_SCAN);
// static const float ang_res_y = 33.2/float(N_SCAN-1);
// static const float ang_bottom = 16.6+0.1;
// static const int groundScanInd = 7;

// Ouster OS1-64
// static const int N_SCAN = 64;
// static const int HORIZONTAL_SCAN = 1024;
// static const float ang_res_x = 360.0/float(HORIZONTAL_SCAN);
// static const float ang_res_y = 33.2/float(N_SCAN-1);
// static const float ang_bottom = 16.6+0.1;
// static const int groundScanInd = 15;
static const float scanPeriod = 0.1;

static const bool loopClosureEnableFlag = false;
static const size_t mappingFrequencyDivider = 5;
static const double mappingProcessInterval = scanPeriod*mappingFrequencyDivider;

static const int systemDelay = 0;
static const int imuQueLength = 200;

static const float sensorMountAngle = 0.0;
static const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
static const int segmentValidPointNum = 5;
static const int segmentValidLineNum = 3;
static const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
static const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


static const int edgeFeatureNum = 2;
static const int surfFeatureNum = 4;
static const int sectionsTotal = 6;
static const float edgeThreshold = 0.1;
static const float surfThreshold = 0.1;
static const float nearestFeatureSearchSqDist = 25;


// Mapping Params
static const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
static const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)
// history key frames (history submap for loop closure)
static const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
static const int   historyKeyframeSearchNum = 25; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
static const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment

static const float globalMapVisualizationSearchRadius = 500.0; // key frames with in n meters will be visualized


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct ProjectionOut
{
  pcl::PointCloud<PointType>::Ptr segmented_cloud;
  pcl::PointCloud<PointType>::Ptr outlier_cloud;
  cloud_msgs::cloud_info seg_msg;
};


struct AssociationOut
{
  pcl::PointCloud<PointType>::Ptr cloud_outlier_last;
  pcl::PointCloud<PointType>::Ptr cloud_corner_last;
  pcl::PointCloud<PointType>::Ptr cloud_surf_last;
  nav_msgs::Odometry laser_odometry;
};

inline void OdometryToTransform(const nav_msgs::Odometry& odometry,
                                float* transform) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odometry.pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  transform[0] = -pitch;
  transform[1] = -yaw;
  transform[2] = roll;

  transform[3] = odometry.pose.pose.position.x;
  transform[4] = odometry.pose.pose.position.y;
  transform[5] = odometry.pose.pose.position.z;
}

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
