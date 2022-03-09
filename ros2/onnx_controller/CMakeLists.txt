cmake_minimum_required(VERSION 3.5)
project(onnx_controller)

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
set(ONNXRUNTIME_ROOTDIR "/usr/local/")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories("include"
   "${ONNXRUNTIME_ROOTDIR}/include" 
   "${ONNXRUNTIME_ROOTDIR}/include/onnxruntime/core/session"
   ${YAML_CPP_INCLUDE_DIRS})
link_directories("${ONNXRUNTIME_ROOTDIR}/lib")

add_executable(onnx_controller src/onnx_controller.cpp)

target_link_libraries(onnx_controller onnxruntime librt.so)
target_include_directories(onnx_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)



install(TARGETS onnx_controller
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(onnx_controller rclcpp sensor_msgs)

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

ament_package()
