cmake_minimum_required(VERSION 3.8)
project(trxbewi_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_package(INSTALL_TO_SHARE config launch camera_info)