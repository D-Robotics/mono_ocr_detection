cmake_minimum_required(VERSION 3.5)
project(mono_ocr_detection)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(hobot_cv REQUIRED)
find_package(OpenCV 4 REQUIRED)

option(SHARED_MEM "using shared mem" ON)
if (${SHARED_MEM})
  message("using shared mem")
  find_package(hbm_img_msgs REQUIRED)
  add_definitions(-DSHARED_MEM_ENABLED)
endif ()

if(PLATFORM_S100)
  message("build platform S100")
  set(SYS_ROOT ${CMAKE_SYSROOT})
  set(PREFIX_PATH s100)
else()
  message("invalid platform, build platform S100 default")
  set(SYS_ROOT ${CMAKE_SYSROOT})
  set(PREFIX_PATH s100)
endif()

message("PREFIX_PATH is " ${PREFIX_PATH})
message("SYS_ROOT is " ${SYS_ROOT})

include_directories(include ${PROJECT_SOURCE_DIR})
include_directories(
  ${SYS_ROOT}/usr/include/opencv/
  ${SYS_ROOT}/usr/include/
  ${SYS_ROOT}/usr/include/hobot/
  ${SYS_ROOT}/usr/hobot/include/
)

link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu/
  ${SYS_ROOT}/usr/lib/
  ${SYS_ROOT}/usr/lib/aarch64-linux-gnu/
  ${SYS_ROOT}/usr/hobot/lib/
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/image_utils.cpp
  src/mono_ocr_detection_node.cpp
  src/post_process/mono_ocr_detection_output_parser.cpp
  src/post_process/local_nms.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_components
  dnn_node
  hobot_cv
  sensor_msgs
  ai_msgs
  cv_bridge
  OpenCV
)

if (${SHARED_MEM})
  ament_target_dependencies(${PROJECT_NAME} hbm_img_msgs)
endif ()

# component 形式加载（tros_container 内与其他节点同进程零拷贝互通）
add_library(${PROJECT_NAME}_component SHARED
  src/image_utils.cpp
  src/mono_ocr_detection_node.cpp
  src/post_process/mono_ocr_detection_output_parser.cpp
  src/post_process/local_nms.cpp
)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  dnn_node
  hobot_cv
  sensor_msgs
  ai_msgs
  cv_bridge
  OpenCV
)
if (${SHARED_MEM})
  ament_target_dependencies(${PROJECT_NAME}_component hbm_img_msgs)
endif ()
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "MonoOcrDetectionNode"
  EXECUTABLE mono_ocr_detection_comp)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_component
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/launch/${PREFIX_PATH}/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/)

ament_package()
