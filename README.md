# Manchester Robotics Challenge 2 - Autonomous Navigation System

## Overview

This project implements an autonomous navigation system for a Puzzlebot using **ROS 2 Humble**. The robot is capable of:

- Following a lane using computer vision.
- Detecting and obeying traffic lights.
- Detecting traffic signs using YOLOv8.
- Handling intersections through zebra crossing detection.
- Executing maneuvers using wheel odometry.
- Managing navigation decisions through a Finite State Machine (FSM).

The system is distributed between a **Jetson board mounted on the robot** and a **laptop**, reducing the computational load on the embedded platform.

---

# System Architecture

## Jetson

The Jetson is responsible for image processing tasks that require direct camera access.

### Nodes running on the Jetson

- `traffic_light_detector`
- `image_compressor`

### Published Topics

| Topic | Type | Description |
|---------|---------|---------|
| `/traffic_light/state` | String | Detected traffic light state |
| `/video_source/yolo/compressed` | CompressedImage | Compressed camera image |

---

## Laptop

The laptop performs navigation, sign detection, odometry estimation, and state management.

### Nodes running on the Laptop

- `line_follower`
- `line_follower_controller`
- `puzzlebot_odometry`

### Additional Node

The traffic sign detector is launched separately:

- `traffic_sign_detector`

### Published Topics

| Topic | Type |
|---------|---------|
| `/line_error` | Float32 |
| `/intersection_detected` | Bool |
| `/traffic_sign/state` | String |
| `/odom` | Odometry |

---

# Finite State Machine

The navigation controller is implemented as a Finite State Machine.

```text
FOLLOW
│
├── WAIT_INTERSECTION
│       │
│       ├── TURN
│       └── STRAIGHT
│
├── SPECIAL
│
└── STOPPED
```

---

## FOLLOW

Default navigation mode.

Responsibilities:

- Follow lane center.
- Detect intersections.
- Receive traffic sign commands.
- React to traffic lights.

---

## WAIT_INTERSECTION

Activated when:

- A zebra crossing is detected.
- A maneuver has already been assigned.

The robot stops and waits before executing the maneuver.

---

## TURN

Used for:

- LEFT
- RIGHT

The robot uses odometry to:

1. Move forward slightly.
2. Rotate 90 degrees.
3. Return to line following.

---

## STRAIGHT

Used when:

- STRAIGHT sign is detected.
- GIVE WAY behavior finishes.

The robot crosses the intersection using odometry before returning to line following.

---

## SPECIAL

Special traffic sign behaviors.

### GIVE WAY

1. Slow down.
2. Stop at zebra crossing.
3. Wait.
4. Move forward through the intersection.

### WORKERS

Reduce speed for a predefined amount of time.

### ROUNDABOUT

Reduce speed while maintaining line following.

---

## STOPPED

Emergency stop mode.

The robot remains stopped until a START command is received.

---

# Line Following

The lane detector uses:

1. Image decompression.
2. Grayscale conversion.
3. Gaussian blur.
4. Otsu thresholding.
5. Morphological filtering.
6. Multi-scanline analysis.

Five weighted scanlines are used near the bottom of the image to estimate the lane center.

The resulting error is published on:

```text
/line_error
```

---

# Zebra Crossing Detection

Intersections are detected by monitoring the amount of white pixels in a Region of Interest located at the bottom of the image.

When the lane disappears due to a zebra crossing, the controller transitions into:

```text
WAIT_INTERSECTION
```

---

# Traffic Light Detection

Traffic lights are detected using:

- HSV color segmentation
- Blob detection
- Circularity filtering

Possible states:

```text
RED
YELLOW
GREEN
UNKNOWN
```

Published topic:

```text
/traffic_light/state
```

---

# Traffic Sign Detection

Traffic signs are detected using a YOLOv8 model.

Supported classes:

```text
LEFT
RIGHT
STRAIGHT
STOP
GIVE_WAY
WORKERS
ROUNDABOUT
```

The detector publishes:

```text
/traffic_sign/state
```

Only detections above the confidence threshold are accepted.

---

# Odometry

Wheel encoder measurements are used to estimate:

- Position
- Orientation

Published topic:

```text
/odom
```

The controller uses odometry for:

- Turning maneuvers
- Crossing intersections
- Straight movements after intersections

---

# Image Compression

To reduce network traffic between the Jetson and the laptop, images are compressed before transmission.

Pipeline:

```text
Raw Image
    ↓
Resize (416x416)
    ↓
Grayscale Conversion
    ↓
JPEG Compression
    ↓
CompressedImage
```

This significantly reduces bandwidth consumption while preserving enough information for line following and traffic sign detection.

---

# Parameters

The main parameters are configured in `params.yaml`.

## Line Follower

```yaml
crop_percent: 0.20
```

---

## Controller

```yaml
linear_speed: 0.15
curve_speed: 0.07

kp_straight: 0.0015
kp_curve: 0.0040
kd: 0.0030

max_angular_vel: 1.5
```

---

## Traffic Light Detector

```yaml
min_blob_area: 2000
min_circularity: 0.95
```

---

## Image Compressor

```yaml
width: 416
height: 416

jpeg_quality: 40

grayscale: true
```

---

# Running the System

## 1. Configure ROS Networking

On both the Jetson and the laptop:

```bash
export ROS_DOMAIN_ID=0
```

Verify communication:

```bash
ros2 topic list
```

---

# Jetson Setup

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Source the workspace:

```bash
source ~/ros2_challenge_mcr2/ros2_challenge2/install/setup.bash
```

Launch the Jetson nodes:

```bash
ros2 launch mcr2_puzzlebot puzzlebot_jetson.launch.py
```

This starts:

- `traffic_light_detector`
- `image_compressor`

---

# Laptop Setup

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Source the workspace:

```bash
source ~/ros2_challenge_mcr2/ros2_challenge2/install/setup.bash
```

Launch navigation:

```bash
ros2 launch mcr2_puzzlebot puzzlebot_lap.launch.py
```

This starts:

- `line_follower`
- `line_follower_controller`
- `puzzlebot_odometry`

---

# Running YOLO Traffic Sign Detection

The traffic sign detector must be launched separately:

```bash
ros2 run mcr2_puzzlebot traffic_sign_detector
```

---

# Useful Debug Commands

Check line follower frequency:

```bash
ros2 topic hz /line_error
```

Check traffic light detections:

```bash
ros2 topic echo /traffic_light/state
```

Check traffic sign detections:

```bash
ros2 topic echo /traffic_sign/state
```

Check odometry:

```bash
ros2 topic echo /odom
```

Check active nodes:

```bash
ros2 node list
```

Check active topics:

```bash
ros2 topic list
```

---

# Project Structure

```text
mcr2_puzzlebot/
│
├── image_compressor.py
├── line_follower.py
├── line_follower_controller.py
├── puzzlebot_odometry.py
├── traffic_light_detector.py
├── traffic_sign_detector.py
│
├── launch/
│   ├── puzzlebot_jetson.launch.py
│   └── puzzlebot_lap.launch.py
│
├── config/
│   └── params.yaml
│
└── models/
    └── YOLOv8 weights
```

---

# Authors

Manchester Robotics Challenge 2

Tecnológico de Monterrey