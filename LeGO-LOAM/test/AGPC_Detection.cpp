#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <memory>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <cmath>
#include <vector>

#include "GRANSAC.hpp"
#include "PlaneModel.hpp"
#include <omp.h>
#include <opencv2/opencv.hpp>

// 定義點結構
struct Point {
    double x, y, z;
};

// 函數用來計算點到平面的距離
double calculateDistance(const Point& p, const std::vector<double>& Gk) {
    return std::abs(Gk[0]*p.x + Gk[1]*p.y + Gk[2]*p.z + Gk[3]) / 
           std::sqrt(Gk[0]*Gk[0] + Gk[1]*Gk[1] + Gk[2]*Gk[2]);
}


std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> ConvertPointCloudToGRANSAC(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    CandPoints.resize(cloud->points.size());

#pragma omp parallel for num_threads(6)
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const pcl::PointXYZ& p = cloud->points[i];
        std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point3D>(p.x, p.y, p.z, i);
        CandPoints[i] = CandPt;
    }

    return CandPoints;
}

// 主要程式碼入口
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("extract_and_detect_ground_plane");

    std::string bag_file = "/home/iec/backup/part1_2024-08-28-18-36-23/part1_2024-08-28-18-36-23.db3"; // rosbag2檔案路徑
    std::string pointcloud_topic = "/velodyne_points"; // 點雲topic

    // 初始化 rosbag2 讀取器
    auto reader = std::make_shared<rosbag2_cpp::Reader>();
    reader->open(bag_file);

    int frame_number = 0;
    int target_frame = 500; // 想要提取的幀號

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    while (reader->has_next()) {
        auto bag_message = reader->read_next();
        // std::cout << bag_message->topic_name << std::endl;
        if (bag_message->topic_name == pointcloud_topic) {
            auto library = rosbag2_cpp::get_typesupport_library("sensor_msgs/msg/PointCloud2", "rosidl_typesupport_cpp");
            auto ts = rosbag2_cpp::get_typesupport_handle("sensor_msgs/msg/PointCloud2", "rosidl_typesupport_cpp", library);

            auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            rclcpp::SerializedMessage extracted_msg(*bag_message->serialized_data);
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
            serializer.deserialize_message(&extracted_msg, pointcloud_msg.get());


            frame_number++;
            if (frame_number == target_frame) {
                pcl::fromROSMsg(*pointcloud_msg, *cloud);
                pcl::io::savePCDFileASCII("extracted_pointcloud.pcd", *cloud);
                std::cout << "Saved point cloud to extracted_pointcloud.pcd" << std::endl;
                break;
            }
        }
    }

    rclcpp::shutdown();

    // 從剛才提取的點雲中轉換成我們的自定義 Point 結構並進行濾波
    std::vector<Point> custom_cloud;
    for (const auto& pcl_point : cloud->points) {
        // std::cout << "z value : " << pcl_point.z << std::endl;
        if (pcl_point.z >= -0.05-0.035-0.05 && pcl_point.z <= 0.05-0.035-0.05) {
            Point pt = {pcl_point.x, pcl_point.y, pcl_point.z};
            custom_cloud.push_back(pt);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 將 custom_cloud 轉換為 PCL 格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& pt : custom_cloud) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = pt.x;
        pcl_point.y = pt.y;
        pcl_point.z = pt.z;
        filtered_cloud->points.push_back(pcl_point);
    }

    // 設置點雲的 width 和 height 屬性
    filtered_cloud->width = filtered_cloud->points.size();  // 點的數量
    filtered_cloud->height = 1;  // 非結構化點雲
    filtered_cloud->is_dense = false;  // 如果點雲可能包含無效點（例如 NaN）

    // 儲存為 .pcd 檔案
    pcl::io::savePCDFileASCII("filtered_pointcloud.pcd", *filtered_cloud);
    std::cout << "Saved filtered point cloud to filtered_pointcloud.pcd" << std::endl;
    ////////////////////////////////////////////////////////////////////////////////////


    std::cout << "1" << std::endl;
    // convert to <GRANSAC::AbstractParameter>
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Qk = ConvertPointCloudToGRANSAC(filtered_cloud);
    std::cout << "2" << std::endl;
    // caculate RANSAC
    GRANSAC::RANSAC<PlaneModel, 3> Estimator;
    Estimator.Initialize(0.1, 100); // Threshold, iterations
    std::cout << "3" << std::endl;
    int64_t start = cv::getTickCount();
	Estimator.Estimate(Qk);
	int64_t end = cv::getTickCount();
    std::cout << "4" << std::endl;
    std::cout << "RANSAC took: " << GRANSAC::VPFloat(end - start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms." << std::endl;

    auto Ak = Estimator.GetBestInliers();

    ////////////////////////////////////////////////////////////////////////////////////
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
            int index = point->m_Index;  // 取出原始点云中的索引
            std::cout << "index = " << index << std::endl;
        }
    }
    
    // 設置點雲的 width 和 height 屬性
    Ground_Plane->width = filtered_cloud->points.size();  // 點的數量
    Ground_Plane->height = 1;  // 非結構化點雲
    Ground_Plane->is_dense = false;  // 如果點雲可能包含無效點（例如 NaN）
    
    // 儲存為 .pcd 檔案
    pcl::io::savePCDFileASCII("Ground_Plane.pcd", *Ground_Plane);
    std::cout << "Saved Ground_Plane point cloud to Ground_Plane.pcd" << std::endl;
    ////////////////////////////////////////////////////////////////////////////////////

    // // 輸出最終的地面參數
    // std::cout << "Ground plane parameters: ";
    // for (double param : Gk_star) {
    //     std::cout << param << " ";
    // }
    // std::cout << std::endl;

    return 0;
}







