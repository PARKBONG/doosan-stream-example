#!/bin/bash

ROBOT_NS="dsr01m1013"

# echo "🔐 Requesting control authority..."
# rosservice call /$ROBOT_NS/system/manage_access_control "{access_control: 1}"

# echo "🤖 Setting robot mode to AUTONOMOUS..."
rosservice call /$ROBOT_NS/system/set_robot_mode "{robot_mode: 1}"

# # echo "🧠 Setting robot system to VIRTUAL..."
# rosservice call /$ROBOT_NS/system/set_robot_system "{robot_system: 0}"

# echo "🛡️  Setting safety mode to AUTONOMOUS + EVENT_MOVE..."
rosservice call /$ROBOT_NS/system/set_safety_mode "{safety_mode: 1, safety_mode_event: 1}"

# rosservice call /$ROBOT_NS/realtime/connect_rt_control "{}"
# rosservice call /$ROBOT_NS/realtime/connect_rt_control "{ip_address: '192.168.137.100', port: 12345}"


# echo "▶️ Starting real-time control mode..."
# rosservice call /$ROBOT_NS/realtime/start_rt_control

# echo "✅ Done. You can now run your RT streaming code."

# rosservice call /$ROBOT_NS/realtime/rt_connect