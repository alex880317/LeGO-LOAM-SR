cmake_minimum_required(VERSION 3.10)

# 項目名稱
project(MyESKFProject)

# 設置C++標準
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 包括Eigen庫
include_directories("/path/to/Eigen")

# 包括nlohmann/json庫
include_directories("/path/to/nlohmann")

# 如果Sophus是自定義安裝的，需要包含其路徑
include_directories("/path/to/Sophus")

# 包括其他自定義庫的路徑
include_directories("/path/to/MatrixUtils")
include_directories("/path/to/MeaCovFromMatlab")

# # 尋找Boost庫（如果需要）
# find_package(Boost REQUIRED)
# include_directories(${Boost_INCLUDE_DIRS})

# 定義可執行文件和相關源文件
add_executable(MyESKFExecutable myESKF.cpp)

# 連結Boost庫（如果使用）
target_link_libraries(MyESKFExecutable ${Boost_LIBRARIES})

# 連結其他必要的庫
