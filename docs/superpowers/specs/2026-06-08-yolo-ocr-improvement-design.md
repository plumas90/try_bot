# Design: YOLO + OCR + Execution Improvement

**Date:** 2026-06-08  
**Project:** TurtleBot3 YOLOv8 Capstone  
**Status:** Approved

---

## Overview

Improve recognition rate and execution convenience of the TurtleBot3 capstone project. Four areas addressed: YOLO detection accuracy, paper detection robustness, OCR reliability, and launch/run workflow simplification.

**Environment:**
- Remote PC + TurtleBot3 (Option B setup)
- RTX 3070 Ti (CUDA available)
- OCR target: English words + numbers only (`laptop`, `chair`, `backpack`, numeric indices)

---

## Section 1: YOLO Detection Improvement

**Problem:** `confidence_threshold = 0.08` causes frequent false positives. GPU is unused. All frames processed unnecessarily.

**Changes to `yolo_lidar_mission_node.py`:**

| Parameter | Before | After |
|-----------|--------|-------|
| `confidence_threshold` | 0.08 | 0.40 |
| `process_every_n_frames` | 1 | 3 |
| YOLO device | CPU (default) | `cuda` (explicit) |
| NMS IoU threshold | not set | 0.45 |

- YOLO model loaded with `model = YOLO(path)` then `model.to('cuda')`
- `detect_target()` passes `iou=0.45` to `model.predict()`
- `confidence_threshold` launch arg default updated to `0.40`
- `process_every_n_frames` launch arg default updated to `3`

---

## Section 2: Paper Detection Improvement

**Problem:** HSV fixed-range white detection fails under variable lighting and shadows.

**Changes to `find_paper()` in `yolo_lidar_mission_node.py`:**

New preprocessing pipeline:
```
grayscale → gaussian blur (5x5) → adaptiveThreshold (blockSize=15, C=4)
→ morphological close (kernel 3x3, iter 2) → contour find
```

- Uses `cv2.ADAPTIVE_THRESH_GAUSSIAN_C` with `cv2.THRESH_BINARY`
- Minimum contour area filter added: `area > 500px` (removes noise)
- Aspect ratio filter kept: `0.5 ~ 5.0`
- **Fallback:** if adaptive threshold finds 0 contours, falls back to existing HSV method
- Existing HSV `find_paper()` logic preserved as `_find_paper_hsv()` private method

---

## Section 3: OCR Improvement

**Problem:** Tesseract has poor accuracy for short English words, no GPU support, no fuzzy matching, and short retry window.

**Changes to `ocr_reader_node.py`:**

### Engine Replacement
- Tesseract → **EasyOCR** (`pip install easyocr`)
- Initialized once at node startup: `easyocr.Reader(['en'], gpu=True)`
- Warm-up call on init to avoid first-call latency

### Preprocessing pipeline (applied to paper crop before OCR):
```
crop → resize 2x → grayscale → sharpen (kernel) → CLAHE contrast
```

### Post-processing filter:
- Ignore results shorter than 3 characters
- Ignore results that are digits-only
- Match against known targets using `difflib.get_close_matches(word, ['chair','laptop','backpack'], n=1, cutoff=0.6)`
- If close match found, return the canonical form (corrects `loptop` → `laptop`)

### Retry logic:
- Before: 8-second single timeout
- After: up to 3 attempts × 4-second interval = 12 seconds total
- Each attempt publishes intermediate result for debug visibility

---

## Section 4: Execution Convenience

**Problem:** Long `ros2 launch` commands, repeated build/source cycle, many parameters scattered across launch file.

### 4a. `config.yaml` — frequently tuned parameters

**File:** `cap/config.yaml`

```yaml
# Mission
target_classes: ["laptop", "chair", "backpack"]

# YOLO
confidence_threshold: 0.40
process_every_n_frames: 3

# Movement
linear_speed: 0.18
angular_speed_max: 0.07
stop_distance: 0.55

# OCR
ocr_timeout: 4.0
ocr_max_retries: 3
```

Launch file reads this yaml as node parameters. CLI overrides still work.

### 4b. `Makefile` — single-command shortcuts

**File:** `cap/Makefile`

```makefile
build:
	colcon build --symlink-install && source install/setup.bash

laptop:
	ros2 launch capstone_bringup motion.launch.py target_class:=laptop

chair:
	ros2 launch capstone_bringup motion.launch.py target_class:=chair

backpack:
	ros2 launch capstone_bringup motion.launch.py target_class:=backpack

clean:
	rm -rf build install log
```

### 4c. `run.sh` — full build + launch wrapper

**File:** `cap/run.sh`

- Usage: `./run.sh laptop` or `./run.sh chair`
- Checks ROS2 environment, prints helpful error if not sourced
- Runs `colcon build --symlink-install`, sources `install/setup.bash`, then launches
- Passes remaining args to launch command

### 4d. Launch file cleanup

- Move rarely-changed parameters to `config.yaml`
- Keep only 10-15 frequently adjusted params as explicit `DeclareLaunchArgument` entries in `motion.launch.py`
- Add `launch_ros.actions.SetParametersFromFile` to load `config.yaml`

---

## File Change Summary

| File | Change Type |
|------|-------------|
| `src/capstone_motion/capstone_motion/yolo_lidar_mission_node.py` | Modify: YOLO cuda, confidence, frame skip, `find_paper()` |
| `src/capstone_ocr/capstone_ocr/ocr_reader_node.py` | Rewrite: EasyOCR, preprocessing, fuzzy match, retry |
| `src/capstone_bringup/launch/motion.launch.py` | Modify: load config.yaml, reduce explicit params |
| `config.yaml` | New file |
| `Makefile` | New file |
| `run.sh` | New file |

---

## Out of Scope

- Korean OCR (English-only as decided)
- Replacing state machine architecture
- Adding new mission states
- Simulation/Gazebo support
