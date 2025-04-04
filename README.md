# Neura Vision Programming Challenge: Edge Detection with ROS

## Overview

This repository implements a robust and modular edge detection system using classical image processing methods (Canny, Sobel, Laplacian, Prewitt, Roberts) and the deep learning-based Holistically-Nested Edge Detection (HED) technique. These methods are implemented in Python to provide flexibility and modularity. The C++ implementation focuses on Canny edge detection only, chosen for its real-time performance. Outputs and visualization videos are located in the `catkin_ws/results/` directory. 

## Directory Structure

```bash
catkin_ws/
├── src/
│   ├── edge_detection/
│   │   ├── src/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── data/
│   │   ├── srv/
│   │   ├── utils/
│   │   └── CMakeLists.txt
└── results/
    ├── basic/
    ├── service_client/
    └── vision_ros/
```
## Preliminary Requirements
1. Install ROS Noetic
2. Create a catkin workspace
3. Clone this repository into the src/ directory of your workspace
4. Ready to work on your code and build. 

    See ROS Tutorials: http://wiki.ros.org/ROS/Tutorials 

5. Download the provided bagfiles from https://drive.google.com/file/d/1AMD9iy0D7eYJkkI8dercBVxeL6faP4Ge/view?usp=drive_link and place it in catkin_ws/src/edge_detection/data/bagfiles.

## Installation

```bash
cd ~/catkin_ws/src
# Clone your repository here
catkin build
source ~/catkin_ws/devel/setup.bash
```

## Docker Setup

To facilitate easy setup and avoid dependency issues, Docker Compose is used for managing the environment.

### Build the Docker Image

```bash
docker compose build
```
### GUI Support (Host System)

Run the following on your host system to allow GUI visualization from Docker:

```bash
xhost +
```
### Start the Container
Docker Compose is configured to mount your workspace and results directories.

```bash
docker compose up
```
### Attach to the Container

Once the container is running, open a new terminal and attach to it:


```bash
docker exec -it <container_name> bash
```
## Testing Environment

### 3D Visualization of Robot 

Visualize the robot model:

```bash
roslaunch edge_detection state_test.launch
```


## Basic Edge Detection

### Python:
For standalone execution using all image processing methods such as Canny, Sobel, Laplacian, Prewitt, Roberts as well as deep learning-based Holistically-Nested Edge Detection (HED) method:

```bash
python3 src/edge_detection/src/edge_detector.py
```
Results are stored in `catkin_ws/results/basic/python/`. Please note that only Canny method is tuned and tested. Other methods are just implementations.


### C++:
```bash
rosrun edge_detection edge_detection_bin
```
> **Note:** The C++ node uses only the Canny edge detection algorithm for performance reasons.


Results are stored in `catkin_ws/results/basic/cpp/`.


## Vision ROS Integration

### C++ Node:
```bash
roslaunch edge_detection edge_detection_cpp.launch
```
- Subscribes to RGB and Depth topics

- Performs real-time Canny edge detection

- Publishes:

    - Overlay image on topic: /edge_points/overlay

    - 3D marker on topic: /edge_points/marker
> ⚡️ This is a single-node C++ implementation optimized for speed.

Recorded output videos of the markers and the robot on Rviz in both C++ and Python for a duration of two loops (one loop with all combined visualization and the next loop without robot model and pointclouds) of the given .bag file are placed in results/vision_ros/


###  Python Nodes: 
#### ROS Service + Client Test
This setup uses a modular Python-based ROS service and a client. 
To test the service with images from directory, launch with:
```bash
roslaunch edge_detection edge_detection_service_test.launch
```
- Custom ROS Service (`srv/EdgeDetection.srv`) for modular use.
- Processed images are saved under `catkin_ws/results/service_client/python/`.

You can specify the edge detection method and thresholds in launch file parameters:
```xml
<param name="method" value="canny" />
<param name="canny_low_threshold"  value="250" />
<param name="canny_high_threshold" value="300" />
```

Valid options: `canny`, `sobel`, `laplacian`, `prewitt`, `roberts`, `hed`
> **Note:** Only Canny method is tuned and tested.


#### Real-Time Service-Client with RViz:
```bash
roslaunch edge_detection edge_detection_service.launch
```
This launch file runs the edge detection service and a client node that subscribes to image topics, calls the service in real-time, and publishes the output overlay image and 3D edge markers for visualization in RViz.

> **Note:** This also allows selecting the method via the `method` parameter in the launch file. But only Canny method is tuned and tested.





