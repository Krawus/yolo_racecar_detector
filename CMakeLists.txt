cmake_minimum_required(VERSION 3.14)
project(yolo_racecar_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()


## FIND PACKAGES NOT AVIABLE VIA ROSDEP
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

if (CUDA_FOUND)
  message("CUDA is available!")
  message("CUDA Libs: ${CUDA_LIBRARIES}")
  message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# FIND TensorRT
list(APPEND PLUGINS "nvinfer")
list(APPEND PLUGINS "nvonnxparser")
list(APPEND PLUGINS "nvparsers")
foreach(libName ${PLUGINS})
    find_library(${libName}_lib NAMES ${libName} "/usr" PATH_SUFFIXES lib)
    list(APPEND PLUGIN_LIBS "${${libName}_lib}")
endforeach()

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --extended-lambda")

include_directories(${PROJECT_NAME} SHARED
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations") #deprecation is not an error
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations") #silent the deprecated methods warnings


ament_auto_add_library(${PROJECT_NAME} SHARED
  include/yolo_racecar_detector/yolo_racecar_detector.hpp
  include/yolo_racecar_detector/visibility_control.hpp
  include/yolo_racecar_detector/engine.hpp
  include/yolo_racecar_detector/yolo_racecar_detector_node.hpp
  include/yolo_racecar_detector/logging.hpp
  include/yolo_racecar_detector/utils.hpp
  include/yolo_racecar_detector/cuda_utils.hpp
  src/yolo_racecar_detector.cpp
  src/engine.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/yolo_racecar_detector/yolo_racecar_detector_node.hpp
  src/yolo_racecar_detector_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "yolo_racecar_detector::YoloRacecarDetectorNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)
  
  target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${PLUGIN_LIBS}
  )


ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  models
  engines
)
