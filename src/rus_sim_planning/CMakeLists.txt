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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(rus_sim_planning
  src/main.cpp
)

# 3. 链接 ROS 2 库（必须）
ament_target_dependencies(rus_sim_planning rclcpp)

# 4. 安装可执行文件（核心！缺失则 ros2 run 找不到）
install(TARGETS rus_sim_planning
  DESTINATION lib/${PROJECT_NAME}  # 必须装到 lib/包名 目录下
)

target_include_directories(rus_sim_planning
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libigl/include}
)
target_link_libraries(rus_sim_planning
  Eigen3::Eigen
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
