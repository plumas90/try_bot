# TurtleBot3 YOLO/OCR Object Mission Project Plan

## 1. Project Overview

본 프로젝트는 TurtleBot3가 카메라, YOLO 객체 탐지, OCR, LiDAR, odometry를 이용해 실내에 배치된 객체를 순차적으로 찾아가는 자율 주행 미션 시스템을 구현하는 것을 목표로 한다.

시연 환경에는 `laptop`, `backpack`, `chair` 세 종류의 객체만 배치한다. 로봇은 YOLO로 현재 목표 객체를 찾고, 객체 정면 근처로 이동한 뒤 객체에 붙어 있는 흰 종이를 찾아 중앙에 맞춰 정지한다. 종이에 적힌 글자를 OCR로 읽어 다음 목표 객체를 결정한다. 종이에 `HOME`이 적혀 있으면 odometry 기반으로 출발지로 복귀하고, 출발지의 HOME 표식을 확인한 뒤 제자리에서 180도 회전하고 정지한다.

## 2. Objectives

- YOLO를 이용해 목표 객체를 탐지한다.
- 탐지된 객체 방향으로 TurtleBot3를 이동시킨다.
- LiDAR를 이용해 전방 장애물과 최소 20 cm 이격 거리를 유지한다.
- 객체 정면에 도착한 뒤, 객체 주변의 흰 종이를 찾아 종이 중앙으로 정렬한다.
- OCR로 종이에 적힌 다음 목표 객체명을 읽는다.
- `HOME`을 읽으면 odometry 경로를 이용해 출발지로 복귀한다.
- 미션 상태와 도착 알림을 ROS2 토픽으로 확인할 수 있게 한다.

## 3. Scope

### Included

- ROS2 Humble 기반 TurtleBot3 제어
- 카메라 이미지 입력 `/image_raw`
- YOLOv8 기반 객체 탐지
- OCR 기반 다음 목표 인식
- LiDAR 기반 전방 장애물 회피
- Odometry 기반 출발지 복귀
- launch 파일을 통한 통합 실행
- 시연용 디버그 토픽 및 이미지 출력

### Excluded

- SLAM
- Nav2
- 지도 기반 경로 계획
- 일반 환경에서의 완전 자율 주행
- 임의 객체 인식

본 프로젝트는 캡스톤 시연 환경에 최적화된 제한된 객체 미션 시스템이다.

## 4. Demonstration Scenario

1. 로봇 주변 또는 시야 내에 첫 목표 객체를 배치한다.
2. 사용자는 첫 목표 객체를 launch argument로 지정한다.
3. 로봇은 YOLO로 목표 객체를 탐지한다.
4. 목표 객체 방향으로 접근한다.
5. 객체 정면에 도착하면 주변 흰 종이를 탐색한다.
6. 종이 중앙으로 정렬하고 OCR 가능한 거리에서 정지한다.
7. OCR로 종이의 글자를 읽는다.
8. 글자가 `chair`, `laptop`, `backpack`이면 해당 객체로 이동한다.
9. 글자가 `HOME`이면 odometry 기록 경로를 따라 출발지로 복귀한다.
10. 출발지 HOME 표식을 OCR로 확인한 후 제자리 180도 회전 후 정지한다.

## 5. System Architecture

```text
TurtleBot3 Sensors
  Camera /image_raw
  LiDAR  /scan
  Odom   /odom
        |
        v
capstone_motion/yolo_lidar_mission_node
  - YOLO object detection
  - paper detection
  - mission state machine
  - obstacle avoidance
  - odom path recording and return
        |
        +--> /cmd_vel
        +--> /target_debug_image
        +--> /mission/state
        +--> /mission/current_target
        +--> /mission/alert
        +--> /motion_debug_text

capstone_ocr/ocr_reader_node
  - enabled by /mission/ocr_enable
  - reads paper text from /image_raw
        |
        +--> /mission/ocr_text
        +--> /ocr_debug_image
```

## 6. ROS2 Packages

### capstone_bringup

통합 launch 패키지이다.

주요 파일:

- `src/capstone_bringup/launch/motion.launch.py`

역할:

- 미션 노드 실행
- OCR 노드 실행
- 주요 파라미터 설정

### capstone_motion

미션 제어 핵심 패키지이다.

주요 파일:

- `src/capstone_motion/capstone_motion/yolo_lidar_mission_node.py`

역할:

- YOLO 모델 로드
- 현재 목표 객체 탐지
- 객체 방향 접근
- 종이 탐색 및 중앙 정렬
- `/cmd_vel` 발행
- LiDAR 기반 장애물 회피
- odometry 기록 및 복귀
- 미션 상태 발행

### capstone_ocr

OCR 패키지이다.

주요 파일:

- `src/capstone_ocr/capstone_ocr/ocr_reader_node.py`

역할:

- `/mission/ocr_enable`이 true일 때만 OCR 수행
- 흰 종이 ROI 추출
- OCR 결과를 `/mission/ocr_text`로 발행

### capstone_perception

보조 perception 패키지이다.

주요 파일:

- `yolo_detector_node.py`
- `target_finder_node.py`
- `paper_finder_node.py`

현재 통합 미션은 `yolo_lidar_mission_node.py` 내부 YOLO/종이 탐지를 중심으로 동작한다.

## 7. Mission State Flow

```text
SEARCH_TARGET
  -> target detected
APPROACH_LOCKED
  -> object front reached
ALIGN_PAPER
  -> paper centered and readable
ARRIVE_HOLD
  -> short stop and alert
READ_NEXT_TARGET
  -> OCR result
    -> chair/laptop/backpack: SEARCH_TARGET
    -> HOME: RETURN_HOME
RETURN_HOME
  -> start position reached
READ_HOME_MARKER
  -> HOME confirmed or timeout
FINAL_TURN_180
  -> DONE
```

베스트케이스 테스트에서는 초기 출발지 루틴을 끄고 바로 `SEARCH_TARGET`에서 시작한다.

## 8. Object Detection Strategy

YOLO COCO 모델은 실제 객체를 다른 label로 오인할 수 있으므로, 시연 객체 세 개로 label을 보정한다.

허용 내부 클래스:

- `laptop`
- `backpack`
- `chair`

alias 예시:

- `tv` -> `laptop`
- `book` -> `laptop`
- `keyboard` -> `laptop`
- `cell phone` -> `laptop`
- `suitcase` -> `backpack`
- `handbag` -> `backpack`
- `couch` -> `chair`
- `dining table` -> `chair`

세 클래스 외 YOLO 결과는 미션 후보에서 제외한다.

## 9. Paper and OCR Strategy

OCR은 항상 객체 접근 후에만 수행한다.

절차:

1. YOLO로 객체를 먼저 찾는다.
2. 객체 정면까지 접근한다.
3. 흰 종이를 찾는다.
4. 종이가 화면 중앙에 오도록 회전한다.
5. 종이가 OCR 가능한 크기가 될 때까지 천천히 접근한다.
6. 정지 후 `/mission/ocr_enable`을 true로 설정한다.
7. OCR 결과에 따라 다음 목표를 결정한다.

OCR 실패 시 바로 복귀하지 않고 다시 종이 정렬 단계로 돌아가 재시도한다.

## 10. Obstacle Avoidance

LiDAR `/scan`을 이용해 전방 장애물을 확인한다.

- 최소 전방 이격 거리: 0.20 m
- 접근 또는 종이 정렬 중 전방 20 cm 이내 장애물이 감지되면 전진을 멈춘다.
- 좌우 LiDAR 영역 중 더 넓은 쪽으로 회전한다.

현재 회피는 제한된 시연 환경용 단순 회피이며, 전역 경로 계획은 사용하지 않는다.

## 11. Odometry Return

미션 중 `/odom`을 계속 수신해 이동 경로를 기록한다.

동작:

1. 노드 시작 후 첫 odom 위치를 home으로 저장한다.
2. 이동 중 일정 거리 이상 움직일 때마다 경로점을 저장한다.
3. OCR 결과가 `HOME`이면 저장된 경로를 역순으로 따라간다.
4. home 근처에 도착하면 출발지 HOME 표식을 OCR로 확인한다.
5. 마지막으로 180도 회전 후 정지한다.

## 12. Main ROS Topics

### Inputs

| Topic | Type | Description |
| --- | --- | --- |
| `/image_raw` | `sensor_msgs/msg/Image` | 카메라 이미지 |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR 거리 |
| `/odom` | `nav_msgs/msg/Odometry` | odometry |
| `/mission/ocr_text` | `std_msgs/msg/String` | OCR 결과 |

### Outputs

| Topic | Type | Description |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | TurtleBot3 속도 명령 |
| `/mission/ocr_enable` | `std_msgs/msg/Bool` | OCR 활성화 |
| `/mission/state` | `std_msgs/msg/String` | 현재 미션 상태 |
| `/mission/current_target` | `std_msgs/msg/String` | 현재 목표 객체 |
| `/mission/alert` | `std_msgs/msg/String` | 도착/완료 알림 |
| `/motion_debug_text` | `std_msgs/msg/String` | 제어 디버그 로그 |
| `/target_debug_image` | `sensor_msgs/msg/Image` | YOLO/종이 디버그 이미지 |
| `/ocr_debug_image` | `sensor_msgs/msg/Image` | OCR 디버그 이미지 |

## 13. Important Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `target_class` | `laptop` from command | 첫 목표 객체 |
| `allowed_target_classes` | `chair,laptop,backpack` | 시연 객체 목록 |
| `confidence_threshold` | `0.08` | YOLO confidence threshold |
| `label_aliases` | launch default | YOLO label 보정 |
| `initial_forward_sec` | `0.0` | 베스트케이스에서는 초기 전진 끔 |
| `search_angular_speed` | `0.35` | 탐색 회전 속도 |
| `linear_speed` | `0.18` | 객체 접근 속도 |
| `locked_blind_forward_speed` | `0.13` | 객체를 잠깐 놓쳤을 때 접근 속도 |
| `object_front_distance` | `0.85` | 객체 정면 도착 거리 기준 |
| `object_close_box_width_ratio` | `0.42` | 객체 박스 크기 기반 도착 기준 |
| `paper_stop_area_ratio` | `0.045` | OCR 정지용 종이 크기 기준 |
| `paper_center_deadband` | `0.28` | 종이 중앙 정렬 허용 범위 |
| `obstacle_stop_distance` | `0.20` | 최소 장애물 이격 거리 |
| `return_stop_distance` | `0.12` | home 복귀 완료 거리 |

## 14. Build and Run

### Build

```bash
cd ~/Workspace/capstone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select capstone_motion capstone_ocr capstone_bringup
source install/setup.bash
```

### TurtleBot Terminal

```bash
cd ~/Workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
ros2 launch my_robot_bringup camera_robot.launch.py
```

### Laptop Terminal

```bash
cd ~/Workspace/capstone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
ros2 launch capstone_bringup motion.launch.py target_class:=laptop
```

### Debug

```bash
ros2 topic echo /motion_debug_text
ros2 topic echo /mission/state
ros2 topic echo /mission/alert
ros2 topic echo /cmd_vel
```

### Emergency Stop

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 15. Test Plan

### Unit-Level Checks

- Python syntax check
- ROS2 package build
- launch parameter loading
- YOLO model path loading
- OCR node import and startup

### Integration Checks

- `/image_raw`, `/scan`, `/odom`, `/cmd_vel` topic visibility
- YOLO debug image output
- OCR debug image output
- mission state transition log
- `/cmd_vel` velocity command output

### Demonstration Tests

1. `laptop` 객체 탐지 및 접근
2. `tv`로 오인된 laptop alias 처리
3. 객체 앞 도착 후 종이 정렬
4. OCR로 `backpack` 읽고 다음 목표 변경
5. OCR로 `chair` 읽고 다음 목표 변경
6. OCR로 `HOME` 읽고 odom 복귀
7. 출발지 HOME 확인 후 180도 회전
8. 전방 장애물 20 cm 이내 회피

## 16. Schedule

| Phase | Work Items | Status |
| --- | --- | --- |
| 1 | ROS2 workspace and package setup | Done |
| 2 | YOLO detection integration | Done |
| 3 | LiDAR stop and obstacle avoidance | In Progress |
| 4 | OCR next-target reading | Done |
| 5 | Object front approach and paper alignment | In Progress |
| 6 | Odometry return home | Done |
| 7 | Best-case demonstration tuning | In Progress |
| 8 | Final presentation and video recording | Pending |

## 17. Risks and Mitigations

| Risk | Impact | Mitigation |
| --- | --- | --- |
| YOLO misclassifies laptop as tv | Target not found | label alias mapping |
| OCR fails because paper is too close | Wrong or missing next target | paper center and area-based stop |
| OCR fails because paper is too far | Missing next target | tune `paper_stop_area_ratio` |
| LiDAR detects object before YOLO box is large | premature paper search | require YOLO box width condition |
| Robot returns after OCR timeout | mission failure | OCR timeout returns to paper alignment |
| Odometry drift | inaccurate return | short demo route and reverse path following |
| Network topic mismatch | no sensor input | `ROS_DOMAIN_ID`, topic check commands |

## 18. Success Criteria

시연 성공 기준은 다음과 같다.

- 첫 목표 객체를 YOLO로 탐지한다.
- 목표 객체 앞까지 이동한다.
- 객체에 붙은 종이 중앙으로 정렬한다.
- OCR로 다음 객체명을 읽는다.
- 읽은 객체로 이동을 반복한다.
- `HOME`을 읽으면 출발지로 복귀한다.
- 출발지에서 HOME 표식을 확인하고 180도 회전 후 정지한다.
- 전체 과정에서 전방 장애물과 약 20 cm 이상 이격한다.

