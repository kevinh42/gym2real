cmake_minimum_required(VERSION 3.13)
project(gym2real)

set(ONNXRUNTIME_ROOTDIR "/usr/local/")
find_package(yaml-cpp REQUIRED)
find_package(urdfdom REQUIRED)

include_directories()
include_directories("include"
    "${ONNXRUNTIME_ROOTDIR}/include" 
    "${ONNXRUNTIME_ROOTDIR}/include/onnxruntime/core/session"
    ${YAML_CPP_INCLUDE_DIRS})
link_directories("${ONNXRUNTIME_ROOTDIR}/lib")


add_executable(onnx_inference onnx_inference.cpp)
target_link_libraries(onnx_inference PRIVATE onnxruntime librt.so ${YAML_CPP_LIBRARIES})