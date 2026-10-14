std_msgs/Header header
string state                    # IDLE ARMING TAKEOFF HOVER NAVIGATING GOAL_REACHED LANDING LANDED ERROR ...
string current_action           # action name being executed ("" when idle)
string message                  # human readable detail / error reason
float32 progress                # 0..1 for the current action
