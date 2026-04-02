cmake_minimum_required(VERSION 3.8)
project(jetson)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(spi_interface
  src/spi_interface.cpp
  include/jetson/crc.cpp
)
ament_target_dependencies(spi_interface
    rclcpp 
    std_msgs 
    sensor_msgs 
    custom_interfaces)

target_include_directories(spi_interface
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(dualsense
    src/dualsense.cpp)

ament_target_dependencies(dualsense
    rclcpp 
    std_msgs 
    sensor_msgs 
    custom_interfaces)

target_include_directories(dualsense
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS
  spi_interface
  dualsense
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
