# ArUco Markers 

This project explores the **use of ArUco markers** in computer vision applications using **OpenCV (C++)**.
It includes generating, detecting, and using markers for **camera calibration** and **augmented reality**.

---

## Overview

The lab is divided into four main parts:

1. **Marker Generation** – create individual ArUco markers and grids using OpenCV.
2. **Marker Detection** – detect markers in real time using a webcam feed.
3. **Camera Calibration** – estimate camera parameters using an ArUco board.
4. **Augmented Reality** – draw 3D objects (axes or cubes) on top of detected markers.

---

## Programs

* `generate_marker.cpp` – generates a single ArUco marker.
* `generate_board.cpp` – creates a grid board of ArUco markers.
* `detect_marker.cpp` – detects markers live from the webcam.
* `pose_estimation.cpp` – estimates marker pose and shows 3D axes.
* `draw_cube.cpp` – overlays a 3D cube on the detected marker.

All programs use OpenCV’s **ArUco module** for detection and pose estimation.

---

## How to Run

### 1. Generate a marker

```bash
./generate_marker -d=16 -id=11 -s=200 -o=marker.png
```

### 2. Detect markers

```bash
./detect_marker -d=16
```

### 3. Calibrate camera

```bash
./calibrate_camera -d=16 -x=5 -y=7 -l=0.04 -s=0.01 -out=output_calibration.yml
```

### 4. Draw cube in AR

```bash
./draw_cube -d=16 -l=0.05 -calib=output_calibration.yml
```

---

## Results

* Markers are detected and tracked in real time.
* The camera calibration file (`output_calibration.yml`) stores intrinsic parameters and distortion coefficients.
* The **pose estimation** program displays a 3D axis aligned with the marker.
* The **draw cube** program overlays a 3D cube that moves with the marker, creating an AR effect.


Would you like me to make it look a bit more **visual (with emojis and better layout)** like a GitHub portfolio-style README, or keep it in this simple academic style?
