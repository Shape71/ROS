cmake_minimum_required(VERSION 3.8)
project(brover)

find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    std_msgs
    controller_manager
    cyphal_ros2_bridge
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_custom_target(brover ALL)
add_custom_command(
    TARGET brover
    COMMAND jsonnet ARGS cyphal_bridge.jsonnet -o cyphal_bridge.json
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/config
)
add_custom_command(
    TARGET brover
    COMMAND jsonnet ARGS cyphal_bridge_base.jsonnet -o cyphal_bridge_base.json
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/config
)

install(
    DIRECTORY urdf launch config
    DESTINATION share/${PROJECT_NAME}
)

set(BROVER_LIB_NAME brover_hardware)

add_library(${BROVER_LIB_NAME} SHARED
    src/brover_system.cpp
)
target_compile_features(${BROVER_LIB_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${BROVER_LIB_NAME}
    cyphal_ros2_bridge
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

target_include_directories(${BROVER_LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${BROVER_LIB_NAME}>
)

pluginlib_export_plugin_description_file(
    hardware_interface
    ${BROVER_LIB_NAME}.xml
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-g -O0)
endif()

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

install(TARGETS ${BROVER_LIB_NAME}
    EXPORT export_${BROVER_LIB_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/brover
  DESTINATION include
)

ament_export_targets(export_${BROVER_LIB_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
