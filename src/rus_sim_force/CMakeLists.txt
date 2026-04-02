cmake_minimum_required(VERSION 3.8)
project(rus_sim_force)

option(ENABLE_TESTING "Include smoke test code in executable" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)

file(GLOB CORE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/rus_sim_force/*.cpp
)

file(GLOB TEST_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/test/src/*.cpp
)

add_library(wrench_estimate STATIC ${CORE_SOURCES})

target_include_directories(wrench_estimate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(wrench_estimate 
  Eigen3::Eigen
)

ament_target_dependencies(wrench_estimate
  rclcpp
  urdf
)


# ── 主程序 ────────────────────────────────────
set(MAIN_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

if(ENABLE_TESTING)
  list(APPEND MAIN_SOURCES ${TEST_SOURCES})
endif()

add_executable(wrench_estimate_node ${MAIN_SOURCES})

target_include_directories(wrench_estimate_node PRIVATE
  # 测试头文件路径：对应 test/test_include/
  ${CMAKE_CURRENT_SOURCE_DIR}/test/test_include
)

target_link_libraries(wrench_estimate_node 
  wrench_estimate
)

ament_target_dependencies(wrench_estimate_node
    rclcpp
)

if(ENABLE_TESTING)
  target_compile_definitions(wrench_estimate_node PRIVATE ENABLE_TESTING)
endif()

# ── 安装规则 ──────────────────────────────────────────────────
install(TARGETS
  wrench_estimate
  wrench_estimate_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}   # ros2 run 从这里找
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
