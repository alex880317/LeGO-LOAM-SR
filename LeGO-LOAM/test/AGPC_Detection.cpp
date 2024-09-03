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

// 定義點結構
struct Point {
    double x, y, z;
};

// 函數用來計算點到平面的距離
double calculateDistance(const Point& p, const std::vector<double>& Gk) {
    return std::abs(Gk[0]*p.x + Gk[1]*p.y + Gk[2]*p.z + Gk[3]) / 
           std::sqrt(Gk[0]*Gk[0] + Gk[1]*Gk[1] + Gk[2]*Gk[2]);
}

// 函數用來擬合平面，這裡假設已經有一個函數來計算平面的參數
std::vector<double> fitPlane(const std::vector<Point>& points) {
    std::vector<double> Gk = {0, 0, 1, 2}; // 假設初始化一個平面參數
    // 在這裡實現擬合算法
    return Gk;
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
    int target_frame = 100; // 想要提取的幀號

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    while (reader->has_next()) {
        auto bag_message = reader->read_next();
        std::cout << bag_message->topic_name << std::endl;
        if (bag_message->topic_name == pointcloud_topic) {
            std::cout << "1" << std::endl;
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
        std::cout << "z value : " << pcl_point.z << std::endl;
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

    // 地面檢測算法
    int t = 0, t_iter = 10;
    std::vector<double> Gk = {0, 0, 1, 2}; // 初始化平面參數
    double Et = std::numeric_limits<double>::infinity();
    std::vector<double> Gk_star = Gk;

    while (t < t_iter) {
        // Step 2.1 隨機選取 T_np 個點
        std::vector<Point> Qk;
        int T_np = 800; // 假設需要選取3個點
        for (int i = 0; i < T_np; ++i) {
            int index = std::rand() % custom_cloud.size();
            Qk.push_back(custom_cloud[index]);
        }

        // 使用選擇的點擬合平面
        Gk = fitPlane(Qk);

        std::vector<Point> Ak;

        // Step 2.2 計算每個點到擬合平面的距離
        double t_dis = 0.2; // 假設閾值
        for (const auto& Rk_i : custom_cloud) {
            double e_k_i = calculateDistance(Rk_i, Gk);
            if (e_k_i < t_dis) {
                Ak.push_back(Rk_i);
            }
        }

    ////////////////////////////////////////////////////////////////////////////////////
    // 將 custom_cloud 轉換為 PCL 格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr Ground_Plane(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (const auto& pt : Ak) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = pt.x;
        pcl_point.y = pt.y;
        pcl_point.z = pt.z;
        Ground_Plane->points.push_back(pcl_point);
    }
    
    // 設置點雲的 width 和 height 屬性
    Ground_Plane->width = filtered_cloud->points.size();  // 點的數量
    Ground_Plane->height = 1;  // 非結構化點雲
    Ground_Plane->is_dense = false;  // 如果點雲可能包含無效點（例如 NaN）
    
    // 儲存為 .pcd 檔案
    pcl::io::savePCDFileASCII("Ground_Plane.pcd", *Ground_Plane);
    std::cout << "Saved Ground_Plane point cloud to Ground_Plane.pcd" << std::endl;
    ////////////////////////////////////////////////////////////////////////////////////

        // Step 2.3 判斷符合條件的點的數量
        int t_a = 50; // 假設 t_a 是給定的閾值
        if (Ak.size() > t_a) {
            Gk = fitPlane(Ak);
            double new_Et = 0.0;

            for (const auto& Rk_i : Ak) {
                new_Et += calculateDistance(Rk_i, Gk);
            }

            if (new_Et < Et) {
                Gk_star = Gk;
                Et = new_Et;
            }
        }

        ++t;
    }

    // 輸出最終的地面參數
    std::cout << "Ground plane parameters: ";
    for (double param : Gk_star) {
        std::cout << param << " ";
    }
    std::cout << std::endl;

    return 0;
}
