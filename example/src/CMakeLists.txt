cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include include/common include/nlohmann)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(low_level_ctrl src/low_level_ctrl.cpp src/common/motor_crc.cpp)
add_executable(read_low_state src/read_low_state.cpp)
add_executable(read_motion_state src/read_motion_state.cpp)
add_executable(read_wireless_controller src/read_wireless_controller.cpp)
#add_executable(record_bag src/record_bag.cpp)
add_executable(sport_mode_ctrl src/sport_mode_ctrl.cpp src/common/ros2_sport_client.cpp)
add_executable(rotate_in_place src/rotate_in_place.cpp src/common/ros2_sport_client.cpp)
add_executable(walk_with_tilt src/walk_with_tilt.cpp src/common/ros2_sport_client.cpp)
add_executable(walk_with_height src/walk_with_height.cpp src/common/ros2_sport_client.cpp)
add_executable(walk_with_sin_height src/walk_with_sin_height.cpp src/common/ros2_sport_client.cpp)

ament_target_dependencies(low_level_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_wireless_controller ${DEPENDENCY_LIST})
#ament_target_dependencies(record_bag ${DEPENDENCY_LIST})
ament_target_dependencies(sport_mode_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(rotate_in_place ${DEPENDENCY_LIST})
ament_target_dependencies(walk_with_tilt ${DEPENDENCY_LIST})
ament_target_dependencies(walk_with_height ${DEPENDENCY_LIST})
ament_target_dependencies(walk_with_sin_height ${DEPENDENCY_LIST})


install(TARGETS 
        low_level_ctrl
        read_low_state
        read_motion_state
        read_wireless_controller
        sport_mode_ctrl
        rotate_in_place
        walk_with_tilt
        walk_with_height
        walk_with_sin_height
        DESTINATION)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

 add_library(common src/common/motor_crc.cpp src/common/ros2_sport_client.cpp) 
            ament_target_dependencies(common PUBLIC unitree_go unitree_api rclcpp) 
            ament_export_targets(common HAS_LIBRARY_TARGET) 
            install(DIRECTORY include/ DESTINATION include) 
            install( 
            TARGETS common 
            EXPORT common 
            LIBRARY DESTINATION lib 
            ARCHIVE DESTINATION lib 
            RUNTIME DESTINATION bin 
            INCLUDES DESTINATION include) 
            ament_package()
