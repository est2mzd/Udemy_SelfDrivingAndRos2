cmake_minimum_required(VERSION 3.8)
project(data_exchanger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include) # includeフォルダをインクルードパスに追加

add_executable(app_a src/app_a.cpp)
ament_target_dependencies(app_a rclcpp std_msgs)

add_executable(app_b src/app_b.cpp)
ament_target_dependencies(app_b rclcpp std_msgs)

install(TARGETS
  app_a
  app_b
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
