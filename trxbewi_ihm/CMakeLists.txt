cmake_minimum_required(VERSION 3.5)
project(trxbewi_ihm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()