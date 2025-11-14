cmake_minimum_required(VERSION 3.5)
project(odometry)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ===== add these =====
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
# ======================

# add the executable targets
add_executable(EKF_data_pub src/EKF_data_pub.cpp)
add_executable(Rviz_data_pub src/Rviz_data_pub.cpp)

# link dependencies for each target (include tf2 packages)
ament_target_dependencies(EKF_data_pub
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
)

ament_target_dependencies(Rviz_data_pub
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
)

# install targets
install(TARGETS
  EKF_data_pub
  Rviz_data_pub
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
