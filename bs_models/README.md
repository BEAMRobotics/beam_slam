The `bs_models` package contains plugins that can be used to produce state estimates for a wide variety of robots.

## Lidar Odometry

## Visual Odometry

The visual odometry sensor model takes in CameraMeasurementMsgs (produced by the Visual Feature Tracker) and a frame initializer (typically inertial odometry) to 1. Localize each frame using existing landmarks and publish odometry, 2. Triangulate new landmarks to add to the global graph along with the reprojection constraints, 3. Trigger Inertial Odometry to create preintegration constraints between keyframes.

## Inertial Odometry

## SLAM Initialization

## Visual Feature Tracker

## Lidar Scan Deskewer

## Gravity Alignment

## Graph Visualization