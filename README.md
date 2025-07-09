# Stream & RTStream example for Doosan Robot

## Tested System:
- Ubuntu 20.04
- ROS Noetic
- m1509 & m1013

## Requirement
### Install Doosan ROS package
```
git clone https://github.com/DoosanRobotics/doosan-robot.git ~/catkin_ws/src/doosan-robot
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Usage
### Launch robot
for simulator
```
roslaunch dsr_launcher dsr_moveit_gazebo.launch model:=m1013 mode:=virtual
```
for real robot
```
roslaunch dsr_launcher dsr_moveit.launch model:=m1013 host:=192.168.137.100 mode:=real
```

### Move Robot to Home Pose
```
rosservice call /dsr01m1013/motion/move_joint "{pos: [0, 0, 90, 0, 90, 0], vel: 10.0, acc: 20.0, time: 0.0, radius: 0.0, mode: 0, blendType: 0, syncType: 0}"
```

### Change safety mode
```
ROBOT_NS="dsr01m1013"
rosservice call /$ROBOT_NS/system/set_robot_mode "{robot_mode: 1}"
rosservice call /$ROBOT_NS/system/set_safety_mode "{safety_mode: 1, safety_mode_event: 1}"
```
### Publish ROS topic
```
/ser/bin/python3 ServoJStream_example.py
```
## Trouble Shooting
### 2.5.7056 waring
reduce AMPLITUTE 
```
[WARN] [1748494653.498929211]:  level : 2
[WARN] [1748494653.498957862]:  group : 5
[WARN] [1748494653.498994272]:  index : 7056
```
### 1.2.1216 alram
ok to be ignored. but you may increase vel or acc, or increase time
```
[INFO] [1752042597.819005204]: [callback OnLogAlarm]
[INFO] [1752042597.819075010]:  level : 1
[INFO] [1752042597.819109633]:  group : 2
[INFO] [1752042597.819140555]:  index : 1216
[INFO] [1752042597.819165791]:  param : [ServoJ]Can't keep target time due to limit velocity or acceleration
[INFO] [1752042597.819195912]:  param : 
[INFO] [1752042597.819222217]:  param : 
```

## Acknowledgement
When the robot enters safety mode through a 7056 warning, the following effects occur:
- Joint lock occurs temporarily and robot position changes (jerk occurs)
- When the joint lock is released, it moves away from the tracking waypoint, causing large forces
This creates a vicious cycle that triggers 7056 warning again.
Therefore, it is important to provide appropriate parameters.
The main parameters in these examples are:

- vel
- acc
- time_sec # topic publish frequency 
- AMPLITUDE
- FREQUENCY

In particular, time, AMPLITUDE, and FREQUENCY are directly related to robot joint velocity/acceleration.
From this, for example, when linearizing the Sin function in the servoj example:
dq = displacement / sec
   ~ (AMPLITUDE) / (time/FREQUENCY) 
   = 2.5 / (0.01/0.1)
   = 25deg/s
Therefore, you must set these parameters properly. 


