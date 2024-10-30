# cmake_minimum_required(VERSION 3.10)
# project(customizeFactor)

# # 包含頭文件目錄
# set(CUSTOMIZE_FACTOR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# # 收集 src 資料夾中的所有 .cpp 檔案，並生成庫
# file(GLOB CUSTOMIZE_FACTOR_SOURCES "src/*.cpp")
# add_library(customizeFactor STATIC ${CUSTOMIZE_FACTOR_SOURCES})  # 或者改為 SHARED 動態庫

# # 將 include 目錄加入到庫的 target 中
# target_include_directories(customizeFactor PUBLIC ${CUSTOMIZE_FACTOR_INCLUDE_DIR})

# # 設定安裝路徑
# install(TARGETS customizeFactor DESTINATION lib)  # 安裝庫
# install(DIRECTORY src/ DESTINATION include/customizeFactor)  # 安裝頭文件



# customizeFactor/CMakeLists.txt
# 將 src 資料夾中的所有 .cpp 檔案加入到變數 CUSTOMIZE_FACTOR_SOURCES 中
file(GLOB CUSTOMIZE_FACTOR_SOURCES "src/*.cpp")

# 將這些檔案加入到父層的執行目標中
set(CUSTOMIZE_FACTOR_SOURCES ${CUSTOMIZE_FACTOR_SOURCES} PARENT_SCOPE)

