# ROS2 Package Configuration for ZED Depth Processing
# This would be your package.xml if you wanted a full ROS2 package

cmake_minimum_required(VERSION 3.5)
project(zed_depth_ros2)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Install Python executables
install(PROGRAMS
  zed_depth_publisher.py
  zed_depth_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()