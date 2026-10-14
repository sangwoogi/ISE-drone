# Vehicle state as seen by px4_bridge (ENU, frame_id = odom). Upper layers never read px4_msgs.
std_msgs/Header header
bool armed
bool in_offboard
bool landed
bool position_valid
string nav_state                # PX4 nav state name, e.g. "OFFBOARD", "AUTO_LAND"
geometry_msgs/Pose pose         # position + orientation (yaw) in odom frame
geometry_msgs/Vector3 velocity  # m/s, odom frame
