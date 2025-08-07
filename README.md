# Tape Detector ROS2 Package

A robust ROS2 Foxy package for detecting retroreflective tape features using LaserScan sensor data. The system employs dual-centroid feature extraction to handle intensity saturation artifacts common with retroreflective materials.

## Algorithm Overview

The detector implements a sophisticated multi-stage processing pipeline:

1. **Range Filtering**: Validates and filters laser points within operational range
2. **High-Intensity Clustering**: Groups retroreflective returns using Euclidean clustering
3. **Dual Feature Extraction**: 
   - HI Centroid: Center of high-intensity distorted points (red markers)
   - LI Feature: Stable low-intensity reference point nearby (green markers)

## Build Instructions

```bash
# Navigate to your ROS2 workspace
cd ~/catkin_tapeDetector

# Build the package
colcon build --packages-select tape_detector_ros2

# Source the workspace
source install/setup.bash
```

## Usage

### Launch with default parameters:
```bash
ros2 launch tape_detector_ros2 tape_detector.launch.py
```

### Launch with custom LaserScan topic:
```bash
ros2 launch tape_detector_ros2 tape_detector.launch.py scan_topic:=/velodyne_scan
```

### Run node directly with parameters:
```bash
ros2 run tape_detector_ros2 tape_detector_node \
  --ros-args \
  -p min_range:=0.5 \
  -p max_range:=20.0 \
  -p intensity_threshold:=150.0
```

## Topics

### Subscribed Topics
- `/scan` (sensor_msgs/msg/LaserScan): Input laser scan data with intensity values

### Published Topics
- `/high_intensity_centroids` (visualization_msgs/msg/MarkerArray): Red sphere markers at tape centers
- `/stable_feature_points` (visualization_msgs/msg/MarkerArray): Green sphere markers at stable reference points

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_range` | double | 0.1 | Minimum valid range (m) |
| `max_range` | double | 30.0 | Maximum valid range (m) |
| `intensity_threshold` | double | 200.0 | Threshold for retroreflective detection |
| `cluster_tolerance` | double | 0.1 | Maximum distance between clustered points (m) |
| `min_cluster_size` | int | 3 | Minimum points to form valid cluster |
| `search_radius` | double | 0.5 | Search radius for LI points around HI centroid (m) |

## Visualization

View the detection results in RViz2:
```bash
rviz2
```

Add the following displays:
- LaserScan on topic `/scan`
- MarkerArray on topic `/high_intensity_centroids` (red spheres)
- MarkerArray on topic `/stable_feature_points` (green spheres)

## Technical Details

### Sensor Fusion Strategy
The dual-centroid approach provides robust feature localization by:
- Using high-intensity returns to identify tape presence
- Extracting stable low-intensity features for precise positioning
- Mitigating saturation artifacts through intelligent clustering

### Performance Optimization
- Efficient Euclidean clustering with early termination
- Single-pass range filtering and coordinate transformation
- Optimized marker lifetime management (0.2s) for real-time visualization

## Troubleshooting

1. **No markers visible**: Check intensity_threshold parameter matches your sensor's intensity range
2. **Too many false detections**: Increase min_cluster_size or intensity_threshold
3. **Missing tape detections**: Decrease cluster_tolerance or search_radius
4. **Frame ID issues**: Ensure your LaserScan publishes with frame_id "laser"