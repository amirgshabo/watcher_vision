cmake_minimum_required(VERSION 3.5)
project(ros2_publisher)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (located under src/ros2_publisher)
ament_python_install_package(${PROJECT_NAME})

# Install scripts (entry points)
install(PROGRAMS
  resource/ros2_publisher
  DESTINATION share/${PROJECT_NAME}
)

# Ensure package.xml is installed
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
