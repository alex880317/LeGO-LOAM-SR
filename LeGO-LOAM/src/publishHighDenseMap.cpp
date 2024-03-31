#include "publishHighDenseMap.h"
#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/Core> 
#include <Eigen/Geometry>

const std::string PARAM_ReMapping = "map_optimization.ros__parameters.ReMapping";

PCDPublisher::PCDPublisher(const std::string &name) : Node(name){
        //---------------------------------------------------------------------------
        // 在这里声明和获取参数
        this->declare_parameter(PARAM_ReMapping);
        if (!this->get_parameter(PARAM_ReMapping, remapping_enabled)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ReMapping.c_str());
            return;
        }
        if (!remapping_enabled){
            return;
        }
        //---------------------------------------------------------------------------
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("high_dense_map", 10);
        std::string pcd_file_path = "/home/iec/colcon_ws/src/LeGO-LOAM-SR/Result/0329HighDenseWithOutDS/denseCloud.pcd"; // 修改为你的.pcd文件路径
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1) //* load the file
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s \n", pcd_file_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %ld data points from %s", cloud->size(), pcd_file_path.c_str());

        transform_map = rotatePointCloud(cloud);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*transform_map, output); // Convert the cloud to ROS message
        output.header.frame_id = "map"; // Set the frame ID to "map" or another appropriate value

        // Publishing the PointCloud2 message
        publisher_->publish(output);
}

pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PCDPublisher::rotatePointCloud(
    const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& input_cloud) {
    // 创建输出点云
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 首先，创建Z轴的旋转矩阵（90度）
    Eigen::Matrix3f R = (Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ())*
                        Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX())).toRotationMatrix();

    // // 然后，创建X轴的旋转矩阵（90度）
    // Eigen::AngleAxisf rotation_x(M_PI / 2, Eigen::Vector3f::UnitX());

    // 组合两个旋转矩阵
    // Eigen::Matrix4f transform = R.matrix();
    Eigen::Matrix4f transform = Eigen::MatrixXf::Identity(4, 4);
    transform.block<3, 3>(0, 0) = R;

    // 使用旋转矩阵变换点云
    pcl::transformPointCloud(*input_cloud, *output_cloud, transform);

    return output_cloud;
}


// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PCDPublisher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }