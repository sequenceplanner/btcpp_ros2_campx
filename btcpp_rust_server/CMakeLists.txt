cmake_minimum_required(VERSION 3.5)
project(btcpp_rust_server)

find_package(ament_cmake REQUIRED)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

include(r2r_cargo.cmake)

# put ros package dependencies here.
r2r_cargo(std_msgs               # just to test that it works
          rcl                    # we need the c ros2 api
          ur_controller_msgs
          btcpp_ros2_interfaces
          rcl_action             # as of r2r 0.1.0, we also need the action api
          rmw_fastrtps_cpp       # (needed to build with RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
          FastRTPS               # (needed to build with RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
         )

# install binaries
install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/colcon/btcpp_rust_server
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY templates
#  DESTINATION share/${PROJECT_NAME})

# we need this for ros/colcon
ament_package()
