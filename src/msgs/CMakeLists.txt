cmake_minimum_required(VERSION 3.8)
project(msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MTi200Data.msg"
  "msg/GeneratedPath.msg"
  "msg/PWMsStamped.msg"
  "msg/SensorsStamped.msg"
  "msg/ThrustsStamped.msg"
  "msg/DVLBeam.msg"
  "msg/DVLData.msg"
  "msg/DVLTarget.msg"
  "msg/DVLVelocity.msg"
  "msg/Detection3DPoints.msg"
  "msg/Detection3DPointsArray.msg"
  DEPENDENCIES geometry_msgs vision_msgs sensor_msgs std_msgs
)

ament_package()
