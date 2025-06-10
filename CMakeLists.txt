cmake_minimum_required(VERSION 3.8)
project(cloudini_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cloudini_lib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(point_cloud_transport REQUIRED)
# find_package(cloudini_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedPointCloud.msg"
  DEPENDENCIES std_msgs
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

include_directories(
  cloudini_lib
  ./include/cloudini_ros2
)


add_executable(compress src/compress.cpp)
ament_target_dependencies(compress rclcpp std_msgs cloudini_lib sensor_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(compress
  ${cpp_typesupport_target}
)


add_executable(decompress src/decompress.cpp)
ament_target_dependencies(decompress rclcpp std_msgs cloudini_lib sensor_msgs)

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(decompress
  ${cpp_typesupport_target}
)



install(TARGETS
  compress
  decompress
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(compress 
  PUBLIC
  cloudini_lib
)

target_include_directories(decompress 
  PUBLIC
  cloudini_lib
)

ament_package()
