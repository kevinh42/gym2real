cmake_minimum_required(VERSION 3.10)
project(gym2real)

#set(ONNXRUNTIME_ROOTDIR "/usr/local/")
#find_package(yaml-cpp REQUIRED)
#find_package(urdfdom REQUIRED)

include_directories(
    include
    /usr/include
)
#include_directories("include"
#    "${ONNXRUNTIME_ROOTDIR}/include" 
#    "${ONNXRUNTIME_ROOTDIR}/include/onnxruntime/core/session"
#    ${YAML_CPP_INCLUDE_DIRS})
#link_directories("${ONNXRUNTIME_ROOTDIR}/lib")

add_library(sensors
    src/imu.cpp
)
target_link_libraries(sensors
    /usr/lib/libi2c.a
)

add_executable(test_imu test/test_imu.cpp)
target_link_libraries(test_imu
    sensors
)

#add_executable(onnx_inference onnx_inference.cpp)
#target_link_libraries(onnx_inference PRIVATE onnxruntime librt.so ${YAML_CPP_LIBRARIES})