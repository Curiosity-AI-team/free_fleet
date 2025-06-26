cmake_minimum_required(VERSION 3.5)
project(free_fleet_client_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if (ament_cmake_FOUND)
  # ————————————————  
  # cpp-httplib via pkg-config  
  # ————————————————  
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(CPPHTTPLIB REQUIRED cpp-httplib)

  # ————————————————  
  # ROS 2 & other deps  
  # ————————————————  
  set(dependencies
    rclcpp
    rclcpp_action
    tf2
    tf2_ros
    nav2_util
    std_srvs
    sensor_msgs
    nav2_msgs
    geometry_msgs
    rmf_fleet_msgs
    free_fleet
    rtabmap_msgs
    nlohmann_json
  )
  foreach(dep ${dependencies})
    find_package(${dep} REQUIRED)
  endforeach()

  # ————————————————  
  # Main executable  
  # ————————————————  
  add_executable(free_fleet_client_ros2
    src/main.cpp
    src/utilities.cpp
    src/client_node.cpp
    src/client_node_config.cpp
  )

  ament_target_dependencies(free_fleet_client_ros2
    ${dependencies}
  )

  target_include_directories(free_fleet_client_ros2
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${CPPHTTPLIB_INCLUDE_DIRS}
  )

  target_link_libraries(free_fleet_client_ros2
      ${CPPHTTPLIB_LIBRARIES}
  )

  # ————————————————  
  # Testing binaries  
  # ————————————————  
  foreach(target
    fake_action_server
    fake_docking_server)
    add_executable(${target} src/tests/${target}.cpp)
    ament_target_dependencies(${target}
      ${dependencies}
    )
  endforeach()

  # ————————————————  
  # Installation  
  # ————————————————  
  install(TARGETS
    free_fleet_client_ros2
    fake_action_server
    fake_docking_server
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()

else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()
