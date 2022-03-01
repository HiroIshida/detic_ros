cmake_minimum_required(VERSION 3.0.2)
project(detic_ros)

find_package(catkin REQUIRED
    std_msgs
    message_generation
)

add_message_files(
    FILES
    SegmentationInfo.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test_node.test)
endif()
