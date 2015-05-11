cmake_minimum_required(VERSION 2.8.3)
project(vision)
find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  cv_bridge
  image_transport
  pcl_ros
  tf
  laser_geometry
  image_geometry
  simtrack_nodes
)

find_package(OpenCV 2.4.8 REQUIRED)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES BT.action
)

add_service_files(
  DIRECTORY srv
  FILES ReceiveCloud.srv StartAggregator.srv CropShelf.srv
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs sensor_msgs# Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS cv_bridge
          image_transport
          pcl_ros
          roscpp
          tf
          image_geometry
          actionlib
          actionlib_msgs
          message_generation
)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# show additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f
  OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME} SOURCES
  ${FILES_LIST}
  ${PROJECT_SOURCE_DIR}/../README.md
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR} 
  ${PROJECT_SOURCE_DIR}../../amazon_challenge_bt/
)


add_executable(shelf_alignment
  src/shelf_alignment.cpp
)

target_link_libraries(shelf_alignment
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)


add_executable(laser_scanner_listener
    src/laser_scanner_listener.cpp
)

target_link_libraries(laser_scanner_listener
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(periodic_cloud_aggregator
    src/periodic_cloud_aggregator.cpp
)

target_link_libraries(periodic_cloud_aggregator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_dependencies(
 periodic_cloud_aggregator
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(periodic_cloud_test_client
    src/periodic_cloud_test_client.cpp
)

target_link_libraries(periodic_cloud_test_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_dependencies(
 periodic_cloud_test_client
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)


add_executable(object_segmentation_node
    src/object_segmentation_node.cpp
    src/object_segmentation.cpp
)

target_link_libraries(object_segmentation_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(segmentation_test_client
    src/segmentation_test_client.cpp
    src/object_segmentation.cpp
)

target_link_libraries(segmentation_test_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(shelf_crop_tool
    src/shelf_crop_tool.cpp
)

target_link_libraries(shelf_crop_tool
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)


#add_executable(cloud_publisher
#    src/cloud_publisher.cpp)

#target_link_libraries(cloud_publisher
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#  ${PCL_LIBRARIES}
#)

#MESSAGE( STATUS "PCL INCLUDES:         " ${PCL_INCLUDE_DIRS} )
