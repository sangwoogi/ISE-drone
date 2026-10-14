cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

find_package(ament_cmake REQUIRED)

# Phase 5+: uncomment and list messages here, then add rosidl deps to package.xml.
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/SemanticObject.msg"
#   "msg/SemanticObjectArray.msg"
#   DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)

ament_package()
