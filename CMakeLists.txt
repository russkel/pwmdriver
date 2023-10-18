cmake_minimum_required(VERSION 3.8)
project(pwmdriver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(fmt REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(pwm_driver src/pwm_driver.cpp src/pwmlib.cpp)
target_include_directories(pwm_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pwm_driver PUBLIC c_std_99 cxx_std_23)  # Require C99 and C++23
ament_target_dependencies(pwm_driver
  rclcpp
  std_msgs
)

target_link_libraries(pwm_driver
  fmt
)

install(TARGETS pwm_driver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
