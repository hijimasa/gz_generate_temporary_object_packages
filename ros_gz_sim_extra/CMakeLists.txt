cmake_minimum_required(VERSION 3.5)

project(ros_gz_sim_extra)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# TODO(CH3): Deprecated. Remove on tock.
if("$ENV{GZ_VERSION}" STREQUAL "" AND NOT "$ENV{IGNITION_VERSION}" STREQUAL "")
  message(DEPRECATION "Environment variable [IGNITION_VERSION] is deprecated. Use [GZ_VERSION] instead.")
  set(ENV{GZ_VERSION} $ENV{IGNITION_VERSION})
endif()

# Edifice
if("$ENV{GZ_VERSION}" STREQUAL "edifice")
  find_package(ignition-math6 REQUIRED)
  set(GZ_MATH_VER ${ignition-math6_VERSION_MAJOR})

  find_package(ignition-transport10 REQUIRED)
  set(GZ_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(GZ_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  find_package(ignition-gazebo5 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo5_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX ignition)

  message(STATUS "Compiling against Gazebo Edifice")
# Garden
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-math7 REQUIRED)
  set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

  find_package(gz-transport12 REQUIRED)
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

  find_package(gz-msgs9 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX gz)

  message(STATUS "Compiling against Gazebo Garden")
# Default to Fortress
else()
  find_package(ignition-math6 REQUIRED)
  set(GZ_MATH_VER ${ignition-math6_VERSION_MAJOR})

  find_package(ignition-transport11 REQUIRED)
  set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX ignition)

  message(STATUS "Compiling against Gazebo Fortress")
endif()

ign_find_package(gflags
    REQUIRED
    PKGCONFIG gflags)
find_package(std_msgs REQUIRED)

add_executable(remove src/remove.cpp)
ament_target_dependencies(remove
  rclcpp
  std_msgs
)
target_link_libraries(remove
  gflags
  ${GZ_TARGET_PREFIX}-math${GZ_MATH_VER}::core
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
)
ament_target_dependencies(remove std_msgs)

add_library(${PROJECT_NAME} SHARED src/Stopwatch.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  remove
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GTest
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  ament_add_gtest_executable(test_stopwatch
    test/test_stopwatch.cpp
  )

  ament_target_dependencies(test_stopwatch rclcpp)

  target_include_directories(test_stopwatch PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_stopwatch
    ${PROJECT_NAME}
  )

  install(
    TARGETS test_stopwatch
    DESTINATION lib/${PROJECT_NAME}
  )
  ament_add_gtest_test(test_stopwatch)
endif()

ament_package()
