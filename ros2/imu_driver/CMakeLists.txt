cmake_minimum_required(VERSION 3.5)
project(imu_driver)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(JetsonGPIO REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  /root/code/gym2real/include
  /usr/include
  ${BOOST_INCLUDE_DIRS}
)

add_executable(imu_driver src/imu_driver.cpp)
target_link_libraries(imu_driver
  JetsonGPIO::JetsonGPIO
  /root/code/gym2real/build/libsensors.a
  /usr/lib/libi2c.a
  ${BOOST_LIBRARIES}
)
ament_target_dependencies(imu_driver rclcpp sensor_msgs)

install(TARGETS
  imu_driver
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()