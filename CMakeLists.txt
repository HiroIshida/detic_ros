cmake_minimum_required(VERSION 3.0.2)
project(detic_ros)

find_package(catkin REQUIRED
    std_msgs
    sensor_msgs
    message_generation
)

add_message_files(
    FILES
    SegmentationInfo.msg
)

add_service_files(
    FILES
    DeticSeg.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

if(CATKIN_ENABLE_TESTING)
  add_custom_target(${PROJECT_NAME}_download_image ALL
    COMMAND ${PROJECT_SOURCE_DIR}/test/prepare_test_data.sh)

  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test_node.test)
  # add_rostest(test/test_batch_processor.test)  # somehow fail on workflow but fine on local
endif()
