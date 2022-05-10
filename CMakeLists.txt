cmake_minimum_required(VERSION 3.5)
project(hobot_mot)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

add_definitions(-DHR_POSIX -DENABLE_IOU2)

message("CMAKE_SYSROOT is " ${CMAKE_SYSROOT})
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_SYSROOT}/usr/include/
  ./deps/iou_based_mot/include/include/
  ./deps/ipc_tracking//include/include/
  ./deps/feat_based_mot/include/include/
  ./deps/hobotlog/include/include/
  ./deps/jsoncpp/include/
  ./deps/hobot/include/
  )

link_directories(
  ./deps/iou_based_mot/lib
  ./deps/ipc_tracking/lib
  ./deps/feat_based_mot/lib
  ./deps/hobotlog/lib
  ./deps/jsoncpp/lib
  ./deps/hobot/lib
)

set(SOURCE_FILES
    src/hobot_mot.cpp
    src/MOTMethod.cpp
    # src/MOT/IOU.cpp
    src/MOT/IOU2.cpp
    #  src/MOT/ReID.cpp
)

set(MOT_LIBRARIES
  rt dl pthread
  hobotthread
  hobotlog 
  ipc_tracking
  feat_based_mot
  iou_based_mot
  hobotsdk
  jsoncpp
)

add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME} 
  ${MOT_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/)

install(
  FILES
  ./deps/iou_based_mot/lib/libiou_based_mot.a
  ./deps/jsoncpp/lib/libjsoncpp.a
  ./deps/hobotlog/lib/libhobotlog.a
  ./deps/feat_based_mot/lib/libfeat_based_mot.a
  ./deps/ipc_tracking/lib/libipc_tracking.a
  ./deps/hobot/lib/libhobotsdk.a
  ./deps/hobot/lib/libhobotthread.a
  ./deps/iou_based_mot/lib/libiou_based_mot.a
  DESTINATION lib/
)

# Install include files
install(
  FILES include/hobot_mot.h include/mot_data_type.h
  DESTINATION include/${PROJECT_NAME}/
)

# Install config
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
