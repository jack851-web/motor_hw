cmake_minimum_required(VERSION 3.8)
project(motor_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# 定义依赖包 (移除 yaml_cpp_vendor)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  std_msgs
  rclpy
  serial
  rclcpp_lifecycle
)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 创建库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/motor_interface.cpp
  src/motor_hw.cpp
)

# 设置包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS}
)

# 链接第三方库（包括 serial 和 yaml-cpp）
target_link_libraries(${PROJECT_NAME}
  ${serial_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# 链接 ROS 2 依赖
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# 导出插件
pluginlib_export_plugin_description_file(hardware_interface motor_hw.xml)

# 安装规则
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 URDF 文件
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 安装 YAML 配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出依赖
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()