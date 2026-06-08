# capstone_ws TurtleBot3 YOLO/OCR mission

Demo flow:

Camera `/image_raw` -> YOLO target detection -> LiDAR `/scan` stop distance -> `/cmd_vel`.
After each target is reached, OCR reads the white paper near the object and uses that text as the next target.

- At start, the robot moves forward briefly, then rotates to search because the destination objects may not be visible from the start point.
- YOLO decides which object direction to approach, then the robot first moves to the front of that object.
- Only after reaching the object front, the robot searches for the attached paper and centers on it for OCR.
- While approaching or centering on paper, LiDAR keeps at least 0.20 m from front obstacles and turns toward the clearer side.
- LiDAR decides when the robot is close enough to stop.
- OCR publishes the next object name from the white paper on `/mission/ocr_text`.
- `/mission/alert` is published when the robot reaches an object.
- If OCR reads `HOME`, odometry `/odom` is used to return to the start.
- At the start, OCR confirms the `HOME` paper, then the robot turns 180 degrees and stops.
- Stop distance is 0.20 m.
- Obstacle stop distance is 0.20 m.
- No SLAM.
- No Nav2.
- No map.
