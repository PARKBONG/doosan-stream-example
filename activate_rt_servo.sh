#!/bin/bash

ROBOT_NS="dsr01m1013"

rosservice call /$ROBOT_NS/system/set_robot_mode "{robot_mode: 1}"
rosservice call /$ROBOT_NS/system/set_safety_mode "{safety_mode: 1, safety_mode_event: 1}"

# RT: https://github.com/DoosanRobotics/doosan-robot/issues/207
# 7056: https://github.com/DoosanRobotics/doosan-robot/issues/128
# rosservice call /$ROBOT_NS/realtime/connect_rt_control "{ip_address: '192.168.137.100', port: 12345}"
# port: https://github.com/DoosanRobotics/doosan-robot/issues/107
rosservice call /$ROBOT_NS/realtime/connect_rt_control "ip_address: '192.168.137.100'"

rosservice call /$ROBOT_NS/realtime/set_rt_control_output "{version: 'v1.0', period: 0.1, loss: 4}"
rosservice call /$ROBOT_NS/realtime/start_rt_control
...
rosservice call /$ROBOT_NS/realtime/stop_rt_control
rosservice call /$ROBOT_NS/realtime/disconnect_rt_control
