cmake_minimum_required(VERSION 3.5)
project(aloam_velodyne)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#list(APPEND CMAKE_PREFIX_PATH "/usr/local")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lidar_slam_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${PCL_DEFINITIONS})

# Add executables
add_executable(ascanRegistration src/scanRegistration.cpp)
ament_target_dependencies(ascanRegistration
  "rclcpp"
  "sensor_msgs"
  "lidar_slam_msgs"
  "pcl_conversions"
  "PCL"
)
target_include_directories(ascanRegistration PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${PCL_INCLUDE_DIRS}
  ${POINT_CLOUD2_ARR_INCLUDE_DIRS}
  )
#${OpenCV_INCLUDE_DIRS}

add_executable(alaserOdometry src/laserOdometry.cpp)
ament_target_dependencies(alaserOdometry
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "lidar_slam_msgs"
  "pcl_conversions"
  "PCL"
  "Eigen3"
)
target_link_libraries(alaserOdometry
    Ceres::ceres)
target_include_directories(alaserOdometry PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(alaserMapping src/laserMapping.cpp)
ament_target_dependencies(alaserMapping
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "lidar_slam_msgs"
  "pcl_conversions"
  "tf2_ros"
  "PCL"
  "Eigen3"
)
target_link_libraries(alaserMapping
    Ceres::ceres)
target_include_directories(alaserMapping PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

#TODO Port kittiHelper to ROS2
#add_executable(kittiHelper src/kittiHelper.cpp)
#ament_target_dependencies(kittiHelper PUBLIC ${dependencies})

# Install executables
install(TARGETS
  ascanRegistration
  alaserOdometry
  alaserMapping
  #kittiHelper
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include DESTINATION include)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
