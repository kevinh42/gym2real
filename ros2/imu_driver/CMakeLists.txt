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

# IMU library
add_library(sensors
    src/imu.cpp
)
target_link_libraries(sensors
    /usr/lib/libi2c.a
)

# add_executable(test_imu test/test_imu.cpp)
# target_link_libraries(test_imu
#     sensors
# )

add_library(imu
  src/imu.cpp
)

add_executable(test_imu test/test_imu.cpp)
target_link_libraries(test_imu
  imu
  /usr/lib/libi2c.a
)

add_executable(imu_driver src/imu_driver.cpp)

target_include_directories(imu_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(imu_driver
  JetsonGPIO::JetsonGPIO
  imu
  /usr/lib/libi2c.a
  ${BOOST_LIBRARIES}
)
install(TARGETS imu_driver
  DESTINATION lib/${PROJECT_NAME})

include_directories(
  include
  /usr/include
  ${BOOST_INCLUDE_DIRS}
)

ament_target_dependencies(imu_driver rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()