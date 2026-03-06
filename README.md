# Simple:Arm

**Simple:Arm** is an intuitive robotic manipulation system that leverages 3D depth-sensing technology to bridge the gap between digital perception and physical action. By utilizing real-time point cloud data, the system allows for precise control and environment-aware movement of a robotic arm.

## ## Key Features

* **Spatial Awareness:** Real-time 3D environment mapping using depth-sensing hardware.
* **Inverse Kinematics (IK):** Smooth, coordinate-based movement translation from camera space to joint space.
* **Object Tracking:** Automated detection and following of targets within the camera's Field of View (FoV).
* **Modular Design:** Easily adaptable for various arm configurations (3-DOF, 6-DOF) and different depth sensors.

---

## ## Architecture Overview

The system operates on a feedback loop between the vision processing unit and the robotic controller:

1. **Perception:** The depth camera captures a 3D point cloud or RGB-D frame.
2. **Transformation:** Coordinates are mapped from the Camera Frame to the Robot Base Frame using a transformation matrix:

$$T_{camera}^{base} = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}$$


3. **Planning:** The IK solver calculates the necessary joint angles.
4. **Execution:** PWM or Serial commands are sent to the arm actuators.

---

## ## Getting Started

### ### Prerequisites

* **Hardware:** * Robotic Arm (e.g., Lewansoul, UFactory, or Custom Build)
* 3D Depth Camera (e.g., Intel RealSense D435, OAK-D, or Microsoft Azure Kinect)


* **Software:**
* Python 3.8+
* OpenCV & NumPy
* [Insert specific Robotics Library, e.g., ROS2, MoveIt, or PyRobot]



### ### Installation

1. Clone the repository:
```bash
git clone https://github.com/username/simple-arm.git
cd simple-arm

```


2. Install dependencies:
```bash
pip install -r requirements.txt

```


3. Calibrate the camera-to-arm offset:
```bash
python calibrate.py

```



---

## ## Usage

To launch the primary control interface:

```bash
python main.py --mode interactive

```

* **--mode manual:** Control via coordinate input.
* **--mode tracking:** The arm will follow the closest object detected by the depth sensor.

---

## ## Configuration

Settings for joint limits, port selection, and camera resolution can be found in `config.yaml`.

| Parameter | Default | Description |
| --- | --- | --- |
| `max_reach` | 300mm | Maximum extension of the arm. |
| `min_depth` | 100mm | Minimum distance for camera focus. |
| `baud_rate` | 115200 | Communication speed for the controller. |

---

## ## Contributing

Contributions are welcome! If you'd like to improve the IK solver or add support for new sensors, please fork the repo and submit a pull request.

## ## License

This project is licensed under the MIT License - see the `LICENSE` file for details.

---

**Would you like me to help you write the "Calibration" section or perhaps generate a Python script for the Inverse Kinematics?**