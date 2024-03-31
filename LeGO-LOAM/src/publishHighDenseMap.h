#ifndef PUBLISHHIGHDENSEMAP_H
#define PUBLISHHIGHDENSEMAP_H


#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class PCDPublisher : public rclcpp::Node
{
public:
    PCDPublisher(const std::string &name);

private:
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> rotatePointCloud(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& input_cloud);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_map;
};
#endif // PUBLISHHIGHDENSEMAP_H


