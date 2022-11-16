cmake_minimum_required(VERSION 2.8.3)
project(calibration_publisher)
set(ROS_VERSION $ENV{ROS_DISTRO})


find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        pcl_ros
        tf
        image_transport
        autoware_msgs
        # cfg depends
        message_generation 
        dynamic_reconfigure
        )

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
 cfg/Calib.cfg
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        pcl_ros
        tf
        image_transport
        autoware_msgs
        message_runtime
)

set(CMAKE_CXX_FLAGS "-O2 -Wall -g ${CMAKE_CXX_FLAGS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)


# calibration_publisher
add_executable(calibration_publisher
        calibration_publisher.cpp
        )

add_dependencies(calibration_publisher
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
        )
link_directories(
        ${PCL_LIBRARY_DIRS}
        )
target_link_libraries(calibration_publisher
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )

# set_default_param
add_executable(set_default_param
        set_default_param.cpp
        )

add_dependencies(set_default_param
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
        )
        
target_link_libraries(set_default_param
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )

install(TARGETS calibration_publisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
