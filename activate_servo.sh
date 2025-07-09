ROBOT_NS="dsr01m1013"
rosservice call /$ROBOT_NS/system/set_robot_mode "{robot_mode: 1}"
rosservice call /$ROBOT_NS/system/set_safety_mode "{safety_mode: 1, safety_mode_event: 1}"