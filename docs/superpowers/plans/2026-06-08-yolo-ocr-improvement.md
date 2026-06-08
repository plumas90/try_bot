# YOLO + OCR + Execution Improvement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Improve YOLO detection accuracy, paper detection robustness, OCR reliability, and execution convenience for the TurtleBot3 capstone project.

**Architecture:** Four independent improvement areas applied to three existing files plus three new files. Each task is self-contained and testable individually. No state machine changes — only the perception and tooling layers are touched.

**Tech Stack:** ROS2 Humble, Python 3.10, ultralytics YOLOv8, EasyOCR, OpenCV, CUDA (RTX 3070 Ti)

---

## File Map

| File | Action | Responsibility |
|------|--------|---------------|
| `config.yaml` | Create | Frequently-tuned parameter defaults |
| `Makefile` | Create | One-command shortcuts for build and launch |
| `run.sh` | Create | Build + source + launch wrapper |
| `src/capstone_bringup/launch/motion.launch.py` | Modify | Reduce explicit args, load config.yaml |
| `src/capstone_motion/capstone_motion/yolo_lidar_mission_node.py` | Modify | CUDA device, confidence 0.40, frame skip 3, adaptive paper detection |
| `src/capstone_ocr/capstone_ocr/ocr_reader_node.py` | Modify | Replace pytesseract with EasyOCR, add preprocessing and fuzzy match |

---

## Task 1: config.yaml — parameter defaults file

**Files:**
- Create: `config.yaml`

- [ ] **Step 1: Create config.yaml in workspace root**

```yaml
# Frequently tuned parameters — edit here instead of the launch command.
# CLI overrides still work: ros2 launch ... confidence_threshold:=0.5

# Mission
target_class: laptop

# YOLO
confidence_threshold: 0.40
process_every_n_frames: 3

# Movement
linear_speed: 0.18
stop_distance: 0.55

# OCR
ocr_timeout_sec: 4.0
ocr_interval_sec: 0.5
```

Save to `/home/kyuncho/Workspace/cap/config.yaml`.

- [ ] **Step 2: Verify file was created**

```bash
cat /home/kyuncho/Workspace/cap/config.yaml
```

Expected: prints the yaml content above.

- [ ] **Step 3: Commit**

```bash
cd /home/kyuncho/Workspace/cap
git add config.yaml
git commit -m "feat: add config.yaml for frequently-tuned parameters"
```

---

## Task 2: run.sh — build + source + launch wrapper

**Files:**
- Create: `run.sh`

- [ ] **Step 1: Create run.sh**

```bash
#!/bin/bash
set -e

TARGET=${1:-laptop}

if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS2 not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "[run.sh] Building workspace..."
colcon build --symlink-install

echo "[run.sh] Sourcing install/setup.bash..."
source install/setup.bash

echo "[run.sh] Launching mission: target=$TARGET"
ros2 launch capstone_bringup motion.launch.py target_class:=$TARGET
```

Save to `/home/kyuncho/Workspace/cap/run.sh`.

- [ ] **Step 2: Make executable**

```bash
chmod +x /home/kyuncho/Workspace/cap/run.sh
```

- [ ] **Step 3: Verify syntax**

```bash
bash -n /home/kyuncho/Workspace/cap/run.sh
echo "syntax OK"
```

Expected: prints `syntax OK` with no errors.

- [ ] **Step 4: Commit**

```bash
cd /home/kyuncho/Workspace/cap
git add run.sh
git commit -m "feat: add run.sh build+source+launch wrapper"
```

---

## Task 3: Makefile — one-command shortcuts

**Files:**
- Create: `Makefile`

- [ ] **Step 1: Create Makefile**

```makefile
.PHONY: build laptop chair backpack clean

build:
	colcon build --symlink-install

laptop:
	./run.sh laptop

chair:
	./run.sh chair

backpack:
	./run.sh backpack

clean:
	rm -rf build install log
```

Save to `/home/kyuncho/Workspace/cap/Makefile`.

Note: indentation in Makefile **must use TAB characters**, not spaces.

- [ ] **Step 2: Verify make targets parse correctly**

```bash
cd /home/kyuncho/Workspace/cap
make --dry-run laptop 2>&1 | head -5
```

Expected: prints `./run.sh laptop` with no "missing separator" error.

- [ ] **Step 3: Commit**

```bash
git add Makefile
git commit -m "feat: add Makefile with laptop/chair/backpack launch shortcuts"
```

---

## Task 4: YOLO — CUDA device, confidence 0.40, frame skip 3, iou 0.45

**Files:**
- Modify: `src/capstone_motion/capstone_motion/yolo_lidar_mission_node.py`
- Modify: `src/capstone_bringup/launch/motion.launch.py`

### 4a. Update parameter defaults in mission node

- [ ] **Step 1: Update confidence_threshold default**

In `yolo_lidar_mission_node.py` line 55, change:
```python
        self.declare_parameter('confidence_threshold', 0.08)
```
to:
```python
        self.declare_parameter('confidence_threshold', 0.40)
```

- [ ] **Step 2: Update process_every_n_frames default**

In `yolo_lidar_mission_node.py` line 56, change:
```python
        self.declare_parameter('process_every_n_frames', 1)
```
to:
```python
        self.declare_parameter('process_every_n_frames', 3)
```

### 4b. Add CUDA to load_model()

- [ ] **Step 3: Move model to CUDA after loading**

In `load_model()` (around line 298), change:
```python
        return YOLO(str(model_path))
```
to:
```python
        model = YOLO(str(model_path))
        try:
            model.to('cuda')
            self.get_logger().info('YOLO running on CUDA')
        except Exception as e:
            self.get_logger().warn(f'CUDA unavailable, using CPU: {e}')
        return model
```

### 4c. Add iou parameter to model inference

- [ ] **Step 4: Pass iou=0.45 to model call in detect_target()**

In `detect_target()` (around line 355), change:
```python
        result = self.model(frame, verbose=False)[0]
```
to:
```python
        result = self.model(frame, verbose=False, iou=0.45)[0]
```

### 4d. Update launch file defaults

- [ ] **Step 5: Update launch file confidence default**

In `motion.launch.py` line 31, change:
```python
        DeclareLaunchArgument('confidence_threshold', default_value='0.08'),
```
to:
```python
        DeclareLaunchArgument('confidence_threshold', default_value='0.40'),
```

- [ ] **Step 6: Add process_every_n_frames to launch file**

In `motion.launch.py`, add after the confidence_threshold line:
```python
        DeclareLaunchArgument('process_every_n_frames', default_value='3'),
```

And in the Node parameters dict, add:
```python
                'process_every_n_frames': LaunchConfiguration('process_every_n_frames'),
```

- [ ] **Step 7: Verify build succeeds**

```bash
cd /home/kyuncho/Workspace/cap
colcon build --symlink-install --packages-select capstone_motion capstone_bringup 2>&1 | tail -10
```

Expected: `Summary: 2 packages finished` with no errors.

- [ ] **Step 8: Commit**

```bash
git add src/capstone_motion/capstone_motion/yolo_lidar_mission_node.py
git add src/capstone_bringup/launch/motion.launch.py
git commit -m "feat: YOLO CUDA device, confidence 0.40, frame skip 3, iou 0.45"
```

---

## Task 5: Paper detection — adaptive threshold with HSV fallback

**Files:**
- Modify: `src/capstone_motion/capstone_motion/yolo_lidar_mission_node.py`

The current `find_paper()` generates contours via fixed HSV thresholds. We extract contour generation into two private methods and let `find_paper()` try adaptive first.

- [ ] **Step 1: Add _get_paper_contours_adaptive() method**

Add this method to `YoloLidarMissionNode` immediately before `find_paper()` (around line 395):

```python
    def _get_paper_contours_adaptive(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(
            blurred, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            blockSize=15, C=4,
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return [c for c in contours if cv2.contourArea(c) > 500]
```

- [ ] **Step 2: Add _get_paper_contours_hsv() method**

Add this method immediately after `_get_paper_contours_adaptive()`:

```python
    def _get_paper_contours_hsv(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 140), (180, 85, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return list(contours)
```

- [ ] **Step 3: Modify find_paper() to use adaptive with HSV fallback**

In `find_paper()`, replace the existing contour generation block (lines 396–404):

**Remove these lines:**
```python
        height, width = frame.shape[:2]
        image_area = float(width * height)
        image_center_x = width / 2.0
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 140), (180, 85, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

**Replace with:**
```python
        height, width = frame.shape[:2]
        image_area = float(width * height)
        image_center_x = width / 2.0
        contours = self._get_paper_contours_adaptive(frame)
        if not contours:
            contours = self._get_paper_contours_hsv(frame)
```

- [ ] **Step 4: Verify build succeeds**

```bash
cd /home/kyuncho/Workspace/cap
colcon build --symlink-install --packages-select capstone_motion 2>&1 | tail -5
```

Expected: `Summary: 1 packages finished` with no errors.

- [ ] **Step 5: Smoke test — verify find_paper() still returns 7-tuple**

```python
# Run this in a python3 shell to verify the return signature is unchanged
import cv2, sys
sys.path.insert(0, 'src/capstone_motion')
import numpy as np

# Create a white rectangle on black background (simulated paper)
frame = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.rectangle(frame, (100, 150), (400, 300), (220, 220, 220), -1)
cv2.putText(frame, 'LAPTOP', (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,0), 3)

# The function is a method, so we just check adaptive contour logic manually
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
binary = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 4)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
large = [c for c in contours if cv2.contourArea(c) > 500]
print(f"Adaptive contours found: {len(large)}")
assert len(large) > 0, "Should find the white rectangle"
print("OK")
```

Run with: `cd /home/kyuncho/Workspace/cap && python3 -c "<above code>"`

Expected: `Adaptive contours found: N` (N > 0) then `OK`.

- [ ] **Step 6: Commit**

```bash
git add src/capstone_motion/capstone_motion/yolo_lidar_mission_node.py
git commit -m "feat: paper detection uses adaptive threshold with HSV fallback"
```

---

## Task 6: OCR — replace pytesseract with EasyOCR

**Files:**
- Modify: `src/capstone_ocr/capstone_ocr/ocr_reader_node.py`

EasyOCR must be installed first: `pip install easyocr`

### What changes

| Before | After |
|--------|-------|
| `pytesseract` | `easyocr.Reader(['en'], gpu=True)` |
| Otsu binary threshold | CLAHE + adaptive threshold |
| Resize ×3 before OCR | Resize ×2 (EasyOCR handles larger images well) |
| SequenceMatcher fuzzy match | `difflib.get_close_matches` with canonical targets |
| Single attempt per interval | Consecutive-confirmation: publish only after 2 matching results |

- [ ] **Step 1: Install EasyOCR**

```bash
pip install easyocr
```

Expected: installs without error. First run will download the English model (~100MB).

- [ ] **Step 2: Write unit test for normalize_text()**

```bash
mkdir -p /home/kyuncho/Workspace/cap/tests
```

Create `/home/kyuncho/Workspace/cap/tests/test_ocr_normalize.py`:

```python
import sys
sys.path.insert(0, 'src/capstone_ocr')

from capstone_ocr.ocr_reader_node import OcrReaderNode

# We can't spin a ROS2 node in a unit test, so test the pure function directly.
# Instantiate just to call normalize_text — we'll mock the rclpy calls.

import unittest
from unittest.mock import patch, MagicMock

class TestNormalizeText(unittest.TestCase):
    def setUp(self):
        with patch('rclpy.node.Node.__init__', return_value=None), \
             patch.object(OcrReaderNode, 'declare_parameter'), \
             patch.object(OcrReaderNode, 'get_parameter', return_value=MagicMock(value='/image_raw')), \
             patch.object(OcrReaderNode, 'create_subscription'), \
             patch.object(OcrReaderNode, 'create_publisher'), \
             patch('easyocr.Reader'):
            self.node = OcrReaderNode.__new__(OcrReaderNode)
            self.node.get_logger = MagicMock(return_value=MagicMock())

    def test_exact_match(self):
        assert self.node.normalize_text('LAPTOP') == 'LAPTOP'
        assert self.node.normalize_text('CHAIR') == 'CHAIR'
        assert self.node.normalize_text('BACKPACK') == 'BACKPACK'

    def test_lowercase_input(self):
        assert self.node.normalize_text('laptop') == 'LAPTOP'

    def test_fuzzy_match(self):
        assert self.node.normalize_text('LOPTOP') == 'LAPTOP'
        assert self.node.normalize_text('CHIAR') == 'CHAIR'

    def test_too_short_returns_empty(self):
        assert self.node.normalize_text('AB') == ''

    def test_digits_only_returns_empty(self):
        assert self.node.normalize_text('123') == ''

    def test_notebook_maps_to_laptop(self):
        assert self.node.normalize_text('NOTEBOOK') == 'LAPTOP'

if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 3: Run test to confirm it fails (pytesseract version)**

```bash
cd /home/kyuncho/Workspace/cap
python3 -m pytest tests/test_ocr_normalize.py -v 2>&1 | tail -20
```

Expected: some tests FAIL or ERROR because the current `normalize_text` logic differs.

- [ ] **Step 4: Rewrite ocr_reader_node.py**

Replace the full content of `src/capstone_ocr/capstone_ocr/ocr_reader_node.py` with:

```python
#!/usr/bin/env python3

import difflib
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

try:
    import easyocr
except ImportError:
    easyocr = None


KNOWN_TARGETS = ['CHAIR', 'BACKPACK', 'LAPTOP', 'NOTEBOOK', 'HOME']
CANONICAL = {'NOTEBOOK': 'LAPTOP'}


class OcrReaderNode(Node):
    def __init__(self):
        super().__init__('ocr_reader_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('debug_image_topic', '/ocr_debug_image')
        self.declare_parameter('ocr_interval_sec', 0.5)

        image_topic = self.get_parameter('image_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value
        self.ocr_interval = float(self.get_parameter('ocr_interval_sec').value)

        self.bridge = CvBridge()
        self.enabled = False
        self.last_ocr_time = 0.0
        self.last_result = ''
        self._prev_result = ''  # for consecutive-confirmation

        if easyocr is None:
            self.get_logger().error('easyocr not installed. Run: pip install easyocr')
            self.reader = None
        else:
            self.get_logger().info('Loading EasyOCR (English, GPU)...')
            self.reader = easyocr.Reader(['en'], gpu=True)
            # warm-up
            dummy = np.zeros((64, 256, 3), dtype=np.uint8)
            self.reader.readtext(dummy)
            self.get_logger().info('EasyOCR ready.')

        self.create_subscription(Bool, '/mission/ocr_enable', self.enable_cb, 10)
        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.text_pub = self.create_publisher(String, '/mission/ocr_text', 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)

    def enable_cb(self, msg):
        next_enabled = bool(msg.data)
        if next_enabled == self.enabled:
            return
        self.enabled = next_enabled
        if self.enabled:
            self.last_result = ''
            self._prev_result = ''
            self.get_logger().info('OCR ENABLED')
        else:
            self.get_logger().info('OCR DISABLED')

    def normalize_text(self, raw: str) -> str:
        """Normalize raw OCR output to a canonical target name or empty string."""
        text = raw.upper().strip()
        # Strip non-alpha characters
        alpha = ''.join(c for c in text if c.isalpha())

        if len(alpha) < 3:
            return ''

        # Exact match
        if alpha in KNOWN_TARGETS:
            return CANONICAL.get(alpha, alpha)

        # Fuzzy match with cutoff 0.6
        matches = difflib.get_close_matches(alpha, KNOWN_TARGETS, n=1, cutoff=0.6)
        if matches:
            return CANONICAL.get(matches[0], matches[0])

        return ''

    def find_paper_roi(self, frame):
        """Find white paper region using adaptive threshold, fallback to center crop."""
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(
            blurred, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            blockSize=15, C=4,
        )
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < float(w * h) * 0.015:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if bw <= 0 or bh <= 0:
                continue
            ratio = bw / float(bh)
            if ratio < 1.0 or ratio > 3.5:
                continue
            if area > best_area:
                best_area = area
                best = (x, y, bw, bh)

        if best is None:
            x = int(w * 0.15)
            y = int(h * 0.25)
            bw = int(w * 0.70)
            bh = int(h * 0.50)
            return frame[y:y + bh, x:x + bw], (x, y, bw, bh), False

        x, y, bw, bh = best
        pad_x = int(bw * 0.05)
        pad_y = int(bh * 0.08)
        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(w, x + bw + pad_x)
        y2 = min(h, y + bh + pad_y)
        return frame[y1:y2, x1:x2], (x1, y1, x2 - x1, y2 - y1), True

    def preprocess_for_ocr(self, roi):
        """Sharpen and enhance contrast for better OCR input."""
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # CLAHE for local contrast enhancement
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        # Sharpen
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)
        # Resize x2
        gray = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        return gray

    def run_ocr(self, preprocessed) -> str:
        if self.reader is None:
            return ''
        results = self.reader.readtext(preprocessed, detail=1)
        for (_bbox, text, conf) in results:
            if conf < 0.3:
                continue
            normalized = self.normalize_text(text)
            if normalized:
                return normalized
        return ''

    def image_cb(self, msg):
        if not self.enabled:
            return
        now = time.time()
        if now - self.last_ocr_time < self.ocr_interval:
            return
        self.last_ocr_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image convert failed: {exc}')
            return

        roi, rect, paper_found = self.find_paper_roi(frame)
        preprocessed = self.preprocess_for_ocr(roi)
        result = self.run_ocr(preprocessed)

        x, y, w, h = rect
        color = (0, 255, 0) if paper_found else (0, 165, 255)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        show = result if result else 'OCR...'
        cv2.putText(frame, show, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 3)

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_pub.publish(out)

        if result:
            # Consecutive-confirmation: publish only when same result seen twice
            if result == self._prev_result:
                self.last_result = result
                self.text_pub.publish(String(data=result))
                self.get_logger().info(f'OCR RESULT (confirmed): {result}')
            else:
                self._prev_result = result
                self.get_logger().info(f'OCR candidate (waiting for confirm): {result}')
        else:
            self._prev_result = ''
            self.get_logger().info('OCR scanning...', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = OcrReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 5: Run normalize_text tests**

```bash
cd /home/kyuncho/Workspace/cap
python3 -m pytest tests/test_ocr_normalize.py -v 2>&1 | tail -20
```

Expected: all 7 tests PASS.

- [ ] **Step 6: Build OCR package**

```bash
cd /home/kyuncho/Workspace/cap
colcon build --symlink-install --packages-select capstone_ocr 2>&1 | tail -5
```

Expected: `Summary: 1 packages finished` with no errors.

- [ ] **Step 7: Smoke test — EasyOCR on a synthetic image**

```python
# Run: python3 -c "<this code>" from /home/kyuncho/Workspace/cap
import cv2, numpy as np, easyocr

reader = easyocr.Reader(['en'], gpu=True)
img = np.ones((120, 400, 3), dtype=np.uint8) * 240  # white background
cv2.putText(img, 'LAPTOP', (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 0), 4)
results = reader.readtext(img, detail=1)
import difflib
KNOWN = ['CHAIR', 'BACKPACK', 'LAPTOP', 'NOTEBOOK', 'HOME']
for (bbox, text, conf) in results:
    print(f"text='{text}' conf={conf:.2f}")
alpha = ''.join(c for c in results[0][1].upper() if c.isalpha()) if results else ''
matches = difflib.get_close_matches(alpha, KNOWN, n=1, cutoff=0.5)
assert matches and matches[0] in ('LAPTOP', 'NOTEBOOK'), f"Expected LAPTOP match, got '{alpha}' -> {matches}"
print("OCR smoke test OK")
```

Run with: `python3 -c "<above code>"`

Expected: prints `text='LAPTOP' conf=...` then `OCR smoke test OK`.

- [ ] **Step 8: Commit**

```bash
cd /home/kyuncho/Workspace/cap
mkdir -p tests
git add src/capstone_ocr/capstone_ocr/ocr_reader_node.py tests/test_ocr_normalize.py
git commit -m "feat: replace pytesseract with EasyOCR, add adaptive paper ROI and consecutive confirmation"
```

---

## Task 7: Launch file cleanup — load config.yaml, reduce explicit args

**Files:**
- Modify: `src/capstone_bringup/launch/motion.launch.py`

The goal is to load `config.yaml` as a parameter source for both nodes, so users edit `config.yaml` instead of typing long CLI args.

- [ ] **Step 1: Add yaml import and SetParametersFromFile**

Replace the full content of `src/capstone_bringup/launch/motion.launch.py`:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def _config_yaml():
    """Return path to config.yaml: workspace root (dev) or install share (installed)."""
    ws_root = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')
    dev_path = os.path.normpath(os.path.join(ws_root, 'config.yaml'))
    if os.path.exists(dev_path):
        return dev_path
    share = get_package_share_directory('capstone_bringup')
    return os.path.join(share, 'config', 'config.yaml')


def generate_launch_description():
    cfg = _config_yaml()

    return LaunchDescription([
        # --- Frequently changed at runtime (CLI override) ---
        DeclareLaunchArgument('target_class', default_value='laptop'),
        DeclareLaunchArgument('model_path', default_value='models/yolov8s.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.40'),
        DeclareLaunchArgument('process_every_n_frames', default_value='3'),
        DeclareLaunchArgument('linear_speed', default_value='0.18'),
        DeclareLaunchArgument('stop_distance', default_value='0.55'),
        DeclareLaunchArgument('ocr_timeout_sec', default_value='4.0'),
        DeclareLaunchArgument('ocr_interval_sec', default_value='0.5'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('return_home', default_value='true'),
        DeclareLaunchArgument('use_ocr_next_target', default_value='true'),
        DeclareLaunchArgument('mission_sequence', default_value='laptop'),

        Node(
            package='capstone_motion',
            executable='yolo_lidar_mission_node',
            name='yolo_lidar_mission_node',
            output='screen',
            parameters=[
                ParameterFile(cfg, allow_substs=True),
                {
                    'target_class': LaunchConfiguration('target_class'),
                    'model_path': LaunchConfiguration('model_path'),
                    'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                    'process_every_n_frames': LaunchConfiguration('process_every_n_frames'),
                    'linear_speed': LaunchConfiguration('linear_speed'),
                    'stop_distance': LaunchConfiguration('stop_distance'),
                    'ocr_timeout_sec': LaunchConfiguration('ocr_timeout_sec'),
                    'image_topic': LaunchConfiguration('image_topic'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                    'return_home': LaunchConfiguration('return_home'),
                    'use_ocr_next_target': LaunchConfiguration('use_ocr_next_target'),
                    'mission_sequence': LaunchConfiguration('mission_sequence'),
                },
            ],
        ),

        Node(
            package='capstone_ocr',
            executable='ocr_reader_node',
            name='ocr_reader_node',
            output='screen',
            parameters=[
                ParameterFile(cfg, allow_substs=True),
                {
                    'image_topic': LaunchConfiguration('image_topic'),
                    'ocr_interval_sec': LaunchConfiguration('ocr_interval_sec'),
                },
            ],
        ),
    ])
```

- [ ] **Step 2: Build bringup package**

```bash
cd /home/kyuncho/Workspace/cap
colcon build --symlink-install --packages-select capstone_bringup 2>&1 | tail -5
```

Expected: `Summary: 1 packages finished` with no errors.

- [ ] **Step 3: Dry-run launch to verify no import errors**

```bash
source install/setup.bash
ros2 launch capstone_bringup motion.launch.py --show-args 2>&1 | head -30
```

Expected: prints available args including `target_class`, `confidence_threshold`, etc. No Python import errors.

- [ ] **Step 4: Commit**

```bash
git add src/capstone_bringup/launch/motion.launch.py
git commit -m "refactor: launch file loads config.yaml, reduces explicit args to 15"
```

---

## Task 8: Final integration check

- [ ] **Step 1: Full workspace build**

```bash
cd /home/kyuncho/Workspace/cap
colcon build --symlink-install 2>&1 | tail -10
```

Expected: `Summary: 4 packages finished` with no errors.

- [ ] **Step 2: Verify run.sh usage**

```bash
source /opt/ros/humble/setup.bash
bash -n run.sh && echo "run.sh syntax OK"
```

Expected: `run.sh syntax OK`.

- [ ] **Step 3: Verify make targets**

```bash
make --dry-run laptop 2>&1
make --dry-run chair 2>&1
make --dry-run backpack 2>&1
```

Expected: each prints `./run.sh <target>` with no errors.

- [ ] **Step 4: Run all unit tests**

```bash
cd /home/kyuncho/Workspace/cap
python3 -m pytest tests/ -v 2>&1
```

Expected: all tests PASS.

- [ ] **Step 5: Final commit**

```bash
git add -A
git status
git commit -m "chore: final integration check — all packages build, tests pass"
```

---

## Quick Reference After Implementation

| Action | Command |
|--------|---------|
| Build workspace | `make build` or `colcon build --symlink-install` |
| Run laptop mission | `make laptop` or `./run.sh laptop` |
| Run chair mission | `make chair` or `./run.sh chair` |
| Tune parameters | Edit `config.yaml`, then relaunch |
| Override one param | `ros2 launch capstone_bringup motion.launch.py confidence_threshold:=0.5` |
