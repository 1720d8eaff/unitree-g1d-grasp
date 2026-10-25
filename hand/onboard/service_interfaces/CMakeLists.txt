cmake_minimum_required(VERSION 3.8)
project(service_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Setangle.srv"
  "srv/Setspeed.srv"
  "srv/Setforce.srv"
  "srv/Setmode.srv"
  
  "srv/Getangleact.srv"
  "srv/Getangleset.srv"
  "srv/Getspeedset.srv"
  "srv/Getforceact.srv"
  "srv/Getforceset.srv"
  "srv/Getcurrentact.srv"
  "srv/Geterror.srv"
  "srv/Gettemp.srv"
  "srv/Setgestureno.srv"
  
  "msg/GetAngleAct1.msg"
  "msg/GetForceAct1.msg"
    
  "msg/SetAngle1.msg"
  "msg/SetForce1.msg"
  "msg/SetSpeed1.msg"
  "msg/TouchData.msg"
  "msg/PalmTouchData.msg"
  "msg/AllTouchData.msg"
    
  DEPENDENCIES  std_msgs
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
