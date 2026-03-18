cmake_minimum_required(VERSION 3.18)
project(rus_sim_planning)
# 设置使用C++17标准，并启用编译命令导出
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 设置包含目录和源文件
set(LIBIGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libigl/include)  # libigl 库目录
set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)
file(GLOB SRC_FILES src/rus_sim_planning/*.cpp)
file(GLOB TEST_FILES ${TEST_DIR}/*.cpp)

# 添加可执行文件
add_executable(rus_sim_planning
  src/main.cpp
  ${SRC_FILES}
  ${TEST_FILES}
)

# 设置 include 目录
target_include_directories(rus_sim_planning PRIVATE
  ${LIBIGL_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${TEST_DIR}
)

# 链接依赖库
ament_target_dependencies(rus_sim_planning
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  std_msgs
)

target_link_libraries(rus_sim_planning
  Eigen3::Eigen
)

install(TARGETS rus_sim_planning
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
