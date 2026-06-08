# Run commands

## TurtleBot terminal

```bash
cd ~/Workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
ros2 launch my_robot_bringup camera_robot.launch.py
```

## Laptop terminal

```bash
cd ~/Workspace/capstone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
ros2 launch capstone_bringup motion.launch.py target_class:=laptop
```

Use `target_class` for the first object. After the robot reaches it, OCR reads the paper and switches to the next object automatically. If the paper says `HOME`, the robot returns using the recorded odom path.

Optional launch arguments:

```bash
ros2 launch capstone_bringup motion.launch.py \
  target_class:=laptop \
  use_ocr_next_target:=true \
  allowed_target_classes:=chair,laptop,backpack \
  confidence_threshold:=0.08 \
  stop_distance:=0.55 \
  initial_forward_sec:=3.5 \
  paper_stop_area_ratio:=0.045
```

`paper_stop_area_ratio` controls where the robot stops for OCR. Increase it if the paper is still too small in the camera, and decrease it if the robot gets too close before stopping.

## Stop immediately

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## Network check

Run these on the laptop after both sides are launched:

```bash
ros2 topic list | grep -E 'image_raw|scan|odom|cmd_vel|ocr|mission'
ros2 topic echo --once /scan
ros2 topic echo --once /image_raw
ros2 topic echo /mission/state
ros2 topic echo /motion_debug_text
ros2 topic echo /mission/alert
```
