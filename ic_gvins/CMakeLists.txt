cmake_minimum_required(VERSION 3.16)
project(ic_gvins)

# Use one language level across the estimator and ROS2 interface.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d video)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB REQUIRED)

find_package(glog CONFIG REQUIRED)

# ROS-independent estimator library.
add_library(ic_gvins_core SHARED
    src/common/gpstime.cc
    src/fileio/fileloader.cc
    src/fileio/filesaver.cc
    src/preintegration/preintegration_base.cc
    src/preintegration/preintegration_earth.cc
    src/preintegration/preintegration_earth_odo.cc
    src/preintegration/preintegration_normal.cc
    src/preintegration/preintegration_odo.cc
    src/tracking/camera.cc
    src/tracking/drawer.cc
    src/tracking/frame.cc
    src/tracking/map.cc
    src/tracking/mappoint.cc
    src/tracking/tracking.cc
    src/ic_gvins.cc
    src/misc.cc
)
target_compile_features(ic_gvins_core PUBLIC cxx_std_20)
target_compile_options(ic_gvins_core PRIVATE -Wall)
target_include_directories(ic_gvins_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ic_gvins_core
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        glog::glog
        yaml-cpp::yaml-cpp
        ${CERES_LIBRARIES}
        TBB::tbb
)

# ROS2 interface node.
add_executable(${PROJECT_NAME}_ros
    src/ros/fusion_ros.cc
    src/ros/drawer_rviz.cc
)
target_compile_features(${PROJECT_NAME}_ros PRIVATE cxx_std_20)
target_compile_options(${PROJECT_NAME}_ros PRIVATE -Wall)
target_link_libraries(${PROJECT_NAME}_ros PRIVATE
    ic_gvins_core
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
)
if(APPLE)
    # ROS exports include Python message libraries unused by this C++ node.
    # Do not load those libraries (which expect a Python host) at startup.
    target_link_options(${PROJECT_NAME}_ros PRIVATE "LINKER:-dead_strip_dylibs")
endif()

# ROS executables live in lib/<package> for `ros2 run`; reusable libraries and
# headers use the conventional lib/ and include/ prefixes.
install(TARGETS ${PROJECT_NAME}_ros
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ic_gvins_core
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/
    DESTINATION include
)
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(ic_gvins_core)
ament_package()
