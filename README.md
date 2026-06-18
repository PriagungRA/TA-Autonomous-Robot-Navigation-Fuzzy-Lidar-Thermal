# Autonomous Robot Navigation Using Fuzzy Logic, LiDAR, and Thermal Camera

## Overview

This repository contains the implementation of an autonomous wheeled robot navigation system based on Artificial Potential Field (APF) and Mamdani Fuzzy Logic using LiDAR and thermal camera information.

The proposed system integrates thermal perception and LiDAR-based obstacle detection through a decision-level fuzzy fusion mechanism to improve navigation performance in dynamic indoor environments.

The research was conducted as part of an undergraduate thesis at the Department of Electrical Engineering, Institut Teknologi Sepuluh Nopember (ITS), Surabaya, Indonesia.

---

## Research Title

**Autonomous Wheeled Robot Navigation Based on Fuzzy Logic Using LiDAR and Thermal Camera**

---

## System Architecture

The navigation framework consists of the following modules:

1. Thermal Image Processing

   * Thermal image acquisition
   * Thermal obstacle extraction
   * Thermal blind-spot detection

2. LiDAR-Based Perception

   * Obstacle detection
   * Corridor width estimation
   * Environmental awareness

3. Fuzzy Decision System

   * Mamdani fuzzy inference
   * Navigation command generation
   * Dynamic behavior adaptation

4. Navigation Layer

   * ROS Navigation Stack
   * Global Planner (Navfn)
   * Local Planner (Artificial Potential Field)

5. Mobile Robot Platform

   * Differential-drive wheeled robot
   * Gazebo simulation environment

---

## Software Requirements

* Ubuntu 20.04
* ROS Noetic
* Gazebo
* Python 3
* OpenCV
* NumPy
* PCL
* RViz

---

## Repository Structure

```text
.
├── src/
│   ├── asr_navigation/
│   ├── fuzzy_thermal_nav/
│   └── ros_sim/
│
├── experiment_data/
│   ├── S1/
│   ├── S2/
│   ├── S3/
│   ├── S4/
│   ├── S5/
│   ├── S6/
│   ├── S7/
│   └── S8/
│
├── figures/
├── README.md
└── LICENSE
```

---

## Experimental Scenarios

The navigation system was evaluated under eight simulation scenarios.

### Baseline Navigation

| Scenario | Description                |
| -------- | -------------------------- |
| S1       | Open environment           |
| S2       | Single obstacle            |
| S3       | Multiple static obstacles  |
| S4       | Multiple dynamic obstacles |

### Fuzzy-Assisted Navigation

| Scenario | Description                                      |
| -------- | ------------------------------------------------ |
| S5       | Open environment with fuzzy navigation           |
| S6       | Single obstacle with fuzzy navigation            |
| S7       | Multiple static obstacles with fuzzy navigation  |
| S8       | Multiple dynamic obstacles with fuzzy navigation |

---

## Running the Simulation

### Launch Gazebo Environment

```bash
roslaunch ros_sim nav_sim.launch
```

### Launch Thermal Processing

```bash
rosrun fuzzy_thermal_nav semantic_thermal.py
```

### Launch Fuzzy Navigation

```bash
rosrun fuzzy_thermal_nav fuzzy_navigation.py
```

### Visualize Results

```bash
rqt_image_view
```

or

```bash
rviz
```

---

## Experimental Results

The proposed fuzzy-based navigation system was compared against a baseline navigation strategy under multiple environmental conditions.

Performance metrics include:

* Navigation success rate
* Travel time
* Path length
* Obstacle clearance
* Collision rate
* Path smoothness

The complete experimental data and figures are available in the `experiment_data` directory.

---

## Citation

If you use this repository for academic purposes, please cite:

```bibtex
@thesis{alfalakhi2026,
  author  = {Priagung Ramadhan Alfalakhi},
  title   = {Autonomous Wheeled Robot Navigation Based on Fuzzy Logic Using LiDAR and Thermal Camera},
  school  = {Institut Teknologi Sepuluh Nopember},
  year    = {2026}
}
```

---

## Author

**Priagung Ramadhan Alfalakhi**
Department of Electrical Engineering
Institut Teknologi Sepuluh Nopember (ITS)
Surabaya, Indonesia

---

## License

This project is released under the MIT License.
