cmake_minimum_required(VERSION 2.8.3)
project(heimdall_detectnode)

set(ROS_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  serialization
)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
  message_generation
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/DetectNode.cfg
)

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

find_package(PCL 1.2 REQUIRED)

add_definitions("-std=c++0x -O3 -s")
set(CMAKE_BUILD_TYPE Debug)

add_service_files(
    FILES
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES bgs
    CATKIN_DEPENDS 
        roscpp 
        rospy 
        std_msgs 
        cv_bridge
        sensor_msgs
        image_transport
        dynamic_reconfigure
)

link_directories(${PCL_LIBRARY_DIRS})

include_directories(include)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_library(dd.lib
    src/dd/merged.cpp
)

add_library(detect_node.lib 
    src/detect_node/merged.cpp
)

add_library(mask.lib
    src/mask/merged.cpp
)

add_library(pcl_detection.lib
    src/pcl_detection/merged.cpp
)

add_library(dd_detection.lib
    src/dd_detection/merged.cpp
)

add_executable(detect_node src/detect_node.cc)

add_dependencies(detect_node ${PROJECT_NAME}_gencfg)

target_link_libraries(mask.lib 
    pcl_detection.lib
    dd_detection.lib
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(pcl_detection.lib 
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(dd_detection.lib 
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(dd.lib 
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(detect_node.lib 
    pcl_detection.lib
    dd_detection.lib
    mask.lib
    dd.lib
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(detect_node
    detect_node.lib
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)


#Creates VIM Syntastic config file for c++:
#Get includes dirs:
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#Remove duplicate enties:
set(syntastic_include_dir_list)
foreach(dir ${dirs})
    list (FIND syntastic_include_dir_list ${dir} _index)
    if (${_index} EQUAL -1)
        list (APPEND syntastic_include_dir_list ${dir})
    endif()
endforeach()
#Output config file:
set(syntastic_config "")
foreach(dir ${syntastic_include_dir_list})
    set(syntastic_config "${syntastic_config}-I${dir}\n")
endforeach()
#Write file:
message(STATUS "${syntastic_config}")
file(WRITE .syntastic_cpp_config "${syntastic_config}")

