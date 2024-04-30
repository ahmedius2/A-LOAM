cmake_minimum_required(VERSION 3.5)
project(aloam_velodyne
    LANGUAGES CUDA CXX C)

set(CMAKE_VERBOSE_MAKEFILE ON)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wno-pedantic)
endif()

#list(APPEND CMAKE_PREFIX_PATH "/usr/local")
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
# NOTE Modify the following with respect to your architecture
# 72 is AGX Xavier
set(CMAKE_CUDA_ARCHITECTURES "72")

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
find_package(time_profiling_spinner REQUIRED)
find_package(CUDAToolkit)

add_subdirectory(./cudaKDTree)

add_definitions(${PCL_DEFINITIONS})

# Add executables
add_executable(ascanRegistration src/scanRegistration.cpp)
ament_target_dependencies(ascanRegistration
  "rclcpp"
  "sensor_msgs"
  "lidar_slam_msgs"
  "pcl_conversions"
  "PCL"
  "time_profiling_spinner"
)
target_include_directories(ascanRegistration PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${PCL_INCLUDE_DIRS}
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
  "time_profiling_spinner"
)
target_link_libraries(alaserOdometry
    Ceres::ceres)
target_include_directories(alaserOdometry PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

#SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_72,code=sm_72 -Wno-deprecated-gpu-targets")


add_library(alaserMappingCuda src/cudaMapping.cu)
set_target_properties(alaserMappingCuda PROPERTIES CUDA_ARCHITECTURES "72")
target_link_libraries(alaserMappingCuda cudaKDTree)
target_include_directories(alaserMappingCuda PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cudaKDTree/cukd>"
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(alaserMapping src/laserMapping.cpp)
# set_target_properties(alaserMapping PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
# set_target_properties(alaserMapping PROPERTIES LINKER_LANGUAGE CUDA)
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
  "time_profiling_spinner"
)
target_link_libraries(alaserMapping
    alaserMappingCuda
    CUDA::cudart
    CUDA::nvToolsExt
    Ceres::ceres)
target_include_directories(alaserMapping PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)

add_executable(mappedPoseBroadcaster src/mappedPoseBroadcaster.cpp)
ament_target_dependencies(mappedPoseBroadcaster
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "Eigen3"
)
target_include_directories(mappedPoseBroadcaster PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
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
  mappedPoseBroadcaster  
  #kittiHelper
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include DESTINATION include)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
