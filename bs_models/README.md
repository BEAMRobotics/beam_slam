The `bs_models` package contains plugins that can be used to produce state estimates for a wide variety of robots.

## Lidar Odometry

## Visual Odometry

The visual odometry sensor model takes in CameraMeasurementMsgs (produced by the Visual Feature Tracker) and a frame initializer (typically inertial odometry) to:
  1. Localize each frame using existing landmarks and publish odometry
  2. Triangulate new landmarks to add to the global graph along with the reprojection constraints
  3. Trigger Inertial Odometry to create preintegration constraints between keyframes.
     
As is the visual odometry sensor model requires a frame initializer and cannot work standalone (must be in VIO).

## Inertial Odometry

Inertial odometry is responsible to reading imu measurements and publishing its odometry using the current bias estimate in the graph for use elsewhere in the system. Additionally, a "trigger" callback is in IO which when published to, will create a new imu preintegration constraint to the timestamp in the message.

## SLAM Initialization

The SLAM initialization sensor model is essentially a full encased mini SLAM task. It has callbacks on all 3 sensor modalities (IMU, LiDAR and Visual). Using all 3 measurement types, it aims to estimate an initial trajectory, imu biases, and visual feature landmark locations to "bootstrap" the main SLAM system. Currently, only LiDAR based initialization is tested and working, Visual based is to come.

## Visual Feature Tracker

The visual feature tracker does not actually add to the main factor graph in any way, it subscribes to the raw image topics and publishes CameraMeasurementMsgs, which contain feature track information. In the future, this sensor model will be refactored outside of this repo into its own standalone ros node, as we do not want to have sensor models that do not contribute to the graph optimzation.

## Lidar Scan Deskewer

## Gravity Alignment

## Graph Visualization
