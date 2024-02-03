# Hand_tracking_manipulator

I am using an external repo of Dynamixel that supports ROS2 Humble
All the custom scripts are under 'manipulator' package
Custom messages and service files are under 'open_manipulator_msgs' package

### Usage
Launch scripts that calculate movement of manipulator from Pose of hand  
```
ros2 launch manipulator support_launch.py
```
Run the perception script that detects the Hand and gives pose values of each joint
```
ros2 run manipulator arm_tracker.py
```

