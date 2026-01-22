cmake_minimum_required(VERSION 3.8)
project(ap1_pnc_sim_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
# - ros
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# - ap1
find_package(ap1_msgs REQUIRED)
# - opengl
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# glad lib
add_library(glad src/glad.c)
target_include_directories(glad PUBLIC include)

# pnc_sim_visualizer executable
add_executable(pnc_sim_visualizer 
  src/main.cpp
  src/glad.c
)
target_include_directories(pnc_sim_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(pnc_sim_visualizer PUBLIC cxx_std_17)
ament_target_dependencies(
  pnc_sim_visualizer
  "rclcpp"
  "std_msgs"
  "nav_msgs"
  "ap1_msgs"
)
target_link_libraries(pnc_sim_visualizer
  glad
  glfw
  OpenGL::GL
)

install(TARGETS
  pnc_sim_visualizer
  DESTINATION lib/${PROJECT_NAME}
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
