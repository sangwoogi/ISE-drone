cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticObject.msg"
  "msg/SemanticObjectArray.msg"
  "msg/VehicleState.msg"
  "msg/MissionStatus.msg"
  "srv/QueryObjects.srv"
  "srv/AskLLM.srv"
  "action/FlightCommand.action"
  "action/Mission.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
