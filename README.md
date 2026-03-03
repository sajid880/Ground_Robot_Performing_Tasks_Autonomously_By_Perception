# Ground-Robot-Performing-Tasks-Autonomously-by-Perception

# Team Members:

- **Sajeed Hussain**
- **Favour Eberechi wobidi**
- **Arham Sajjad**

# PART 1 Autonomous Driving of a Ground Differential Robot by Perception.

## Introduction

This workspace contains a comprehensive ROS 2 implementation for TurtleBot3 robotic platform development, including autonomous driving capabilities, SLAM mapping, navigation systems, and data logging utilities. The workspace integrates multiple packages for robot control, perception, planning, and mission execution.

The project implements four main functional areas: camera calibration for visual perception, autonomous driving using lane detection and following, simultaneous localization and mapping (SLAM) for environment mapping, and navigation using Nav2 framework for path planning and obstacle avoidance.

## Package Descriptions

### AutoRace Packages

#### turtlebot3_autorace_camera

This package handles camera calibration and image preprocessing for autonomous racing applications. It implements both intrinsic and extrinsic camera calibration procedures to correct lens distortions and establish proper perspective transformations.

Camera calibration is a fundamental prerequisite for accurate visual perception in autonomous robotic systems. The TurtleBot3 Burger platform utilizes a Raspberry Pi Camera Module v2 as its primary vision sensor. Raw camera images inherently contain geometric distortions caused by lens imperfections and sensor misalignments. Without proper calibration, these distortions introduce measurement errors that significantly degrade the performance of computer vision algorithms, particularly those relying on precise geometric relationships such as lane detection, obstacle estimation, and spatial mapping.

The camera calibration process consists of two distinct stages:

**Intrinsic Calibration:** This stage characterizes the internal camera parameters including focal length, principal point (optical center), and lens distortion coefficients. The calibration is performed using the standard ROS 2 camera calibration package with checkerboard calibration patterns. Multiple images of the calibration pattern at various orientations and positions are captured to compute accurate camera matrix and distortion parameters.

**Extrinsic Calibration:** This stage establishes the geometric relationship between the camera coordinate frame and the ground plane. Through extrinsic calibration, the system defines the perspective transformation parameters that convert the forward-facing camera view into a bird's eye view projection.

**Python Modules:**

- **image_compensation.py:** This module improves image quality by adjusting brightness and contrast automatically. When the camera captures images, they might be too dark, too bright, or lack contrast depending on lighting conditions. This code fixes these problems.

The code works by analyzing the image histogram, which shows how bright or dark the pixels are. It finds the darkest and brightest pixels in the image, then stretches the brightness range to use the full available range. This makes dark areas brighter and bright areas more defined, improving overall image visibility.

The process involves converting the image to grayscale, calculating a histogram of pixel brightness values, finding the minimum and maximum brightness levels, and then applying a mathematical transformation to stretch the brightness range. There is a parameter called clip_hist_percent that controls how much of the histogram to clip from the edges, which prevents extreme values from affecting the result too much.

This improved image quality is important because better images make it easier for the lane detection algorithms to work correctly in different lighting conditions.

- **image_projection.py:** This module transforms the camera view from the robot's perspective into a bird's eye view, which is like looking down at the road from above. This transformation is necessary because detecting lanes is much easier when you see the road from the top rather than from the front of the robot.

The code does this by defining a trapezoid shape on the original camera image. This trapezoid represents the area on the road that the robot cares about. The trapezoid has four corner points - two points at the top (closer to the robot) and two points at the bottom (further away from the robot). This creates a trapezoid shape because the road appears narrower when closer and wider when further away due to perspective.

The code then uses a mathematical technique called homography to transform this trapezoid into a rectangle when viewed from above. This creates the bird's eye view effect. The transformation takes the four corner points of the trapezoid in the original image and maps them to four corner points of a rectangle in the output image.

During calibration mode, the code displays red lines on the camera feed showing exactly where the trapezoid boundaries are located. This allows users to adjust the top_x, top_y, bottom_x, and bottom_y parameters to align the trapezoid perfectly with the road surface. The offset_x and offset_y parameters allow fine-tuning the center position of the trapezoid.

Once calibrated, the code applies this transformation to every camera frame, producing a top-down view of the road that makes lane detection much more straightforward and accurate.


**Excuted Commands:**

Launch intrinsic camera calibration:
```bash
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
```

Launch extrinsic camera calibration for image projection:
```bash
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
```

Run image compensation node:
```bash
ros2 run turtlebot3_autorace_camera image_compensation
```

Run image projection node:
```bash
ros2 run turtlebot3_autorace_camera image_projection
```

**Extrinsic Camera Calibration and Image Processing Pipeline:**

The extrinsic calibration process focuses on defining the relationship between the camera and the ground plane for perspective transformation. The calibration launch file (extrinsic_camera_calibration.launch.py) starts two key nodes working together: image_compensation and image_projection.

When the extrinsic calibration launch file is executed, it initializes both nodes with calibration mode enabled. The launch file loads parameter files from the calibration directory (compensation.yaml and projection.yaml) and sets up topic remappings. The image flow starts with rectified camera images from the camera/image_rect_color topic, which first pass through the image_compensation node, then through the image_projection node.

**Image Compensation Node:**

The image_compensation node receives camera images and performs adaptive histogram equalization to enhance image quality. The node subscribes to either compressed or raw image formats from the camera/image_input topic. When calibration mode is active, it allows real-time adjustment of the clip_hist_percent parameter which controls the histogram clipping percentage. This parameter determines how much of the histogram extremes to ignore when stretching the brightness range.

The compensation algorithm works by converting the input image to grayscale, calculating a histogram of pixel intensities, and determining the effective brightness range. If clip_hist_percent is zero, it uses the full dynamic range from minimum to maximum pixel values. Otherwise, it clips a percentage from both ends of the histogram to ignore extreme outliers. The algorithm then calculates scaling factors (alpha and beta) to map the clipped range to the full 0-255 brightness range, applying this transformation using OpenCV's convertScaleAbs function. The enhanced image is published to camera/image_compensated topic, providing improved contrast for subsequent processing stages.

**Image Projection Node:**

The image_projection node performs perspective transformation to convert the forward-facing camera view into a bird's eye view. This transformation is crucial for lane detection as it removes perspective distortion and presents the road surface as a flat plane viewed from above.

The node defines a trapezoidal region of interest using six calibration parameters:
- top_x and top_y: Define the width and vertical position of the trapezoid's top edge (closer to robot)
- bottom_x and bottom_y: Define the width and vertical position of the trapezoid's bottom edge (further from robot)
- offset_x and offset_y: Fine-tune adjustments to the center position of the trapezoid

The trapezoid shape naturally occurs because roads appear narrower when closer to the camera and wider when further away due to perspective projection. The four corner points of the trapezoid are calculated relative to the image center with offsets applied. These source points are mapped to destination points defining a rectangular output region (200x0, 800x0, 800x600, 200x600 pixels), creating a 1000x600 pixel bird's eye view image.

The transformation uses OpenCV's findHomography function which calculates a 3x3 homography matrix that maps points from the trapezoid in the source image to the rectangle in the destination image. This matrix is then applied using warpPerspective to transform every incoming camera frame.

**Intrinsic Calibration Mathematics:**

The camera intrinsic calibration models the relationship between 3D world points and 2D image points using the pinhole camera model. The camera matrix K contains the intrinsic parameters:

```
K = [fx  0  cx]
    [0  fy  cy]
    [0   0   1]
```

where:
- fx, fy: focal lengths in pixels (horizontal and vertical)
- cx, cy: principal point coordinates (optical center)

The projection of a 3D point P = [X, Y, Z]^T to image coordinates [u, v]^T is given by:

```
[u]     [fx  0  cx] [X]
[v] = K [0  fy  cy] [Y]
[w]     [0   0   1] [Z]
```

Normalized coordinates are obtained as: u' = u/w, v' = v/w

Lens distortion is modeled using radial and tangential distortion coefficients. The distortion model corrects image coordinates:

```
x_distorted = x(1 + k1*r² + k2*r⁴ + k3*r⁶) + 2*p1*x*y + p2*(r² + 2*x²)
y_distorted = y(1 + k1*r² + k2*r⁴ + k3*r⁶) + p1*(r² + 2*y²) + 2*p2*x*y
```

where:
- r² = x² + y² (distance from optical center)
- k1, k2, k3: radial distortion coefficients
- p1, p2: tangential distortion coefficients
- x, y: normalized image coordinates before distortion
- x_distorted, y_distorted: distorted image coordinates

The calibration process estimates these parameters using multiple views of a known calibration pattern (checkerboard), minimizing the reprojection error between detected and projected corner points.

**Homography Transformation Mathematics:**

A homography is a projective transformation that maps points from one plane to another. Given a point p = [x, y, 1]^T in the source image and point p' = [x', y', 1]^T in the destination image, the homography transformation is:

```
p' = H · p

[x']   [h11  h12  h13] [x]
[y'] = [h21  h22  h23] [y]
[1 ]   [h31  h32  h33] [1]
```

The homography matrix H has 8 degrees of freedom (9 elements minus scale factor). In normalized coordinates:

```
x' = (h11·x + h12·y + h13) / (h31·x + h32·y + h33)
y' = (h21·x + h22·y + h23) / (h31·x + h32·y + h33)
```

The four corner points of the source trapezoid are defined as:

```
P₁ = [cx - top_x,     cy - top_y]     (top-left)
P₂ = [cx + top_x,     cy - top_y]     (top-right)
P₃ = [cx + bottom_x,  cy + bottom_y]  (bottom-right)
P₄ = [cx - bottom_x,  cy + bottom_y]  (bottom-left)
```

where cx = w_img/2 + offset_x and cy = h_img/2 + offset_y represent the center point with offsets.

These source points are mapped to destination rectangle corners:

```
D₁ = [200, 0]    (destination top-left)
D₂ = [800, 0]    (destination top-right)
D₃ = [800, 600]  (destination bottom-right)
D₄ = [200, 600]  (destination bottom-left)
```

The homography matrix H is computed using the Direct Linear Transform (DLT) algorithm, which solves the system of equations:

```
A · h = 0
```

where h is the vectorized homography matrix and A is constructed from point correspondences. The solution is found using Singular Value Decomposition (SVD), minimizing the algebraic error.

Once computed, the homography matrix transforms every pixel in the source image to the bird's eye view using the perspective transformation:

```
u' = (h₁₁·u + h₁₂·v + h₁₃) / (h₃₁·u + h₃₂·v + h₃₃)
v' = (h₂₁·u + h₂₂·v + h₂₃) / (h₃₁·u + h₃₂·v + h₃₃)
```

This transformation effectively removes perspective distortion, converting the trapezoidal road region into a rectangular bird's eye view that simplifies lane detection and tracking algorithms.

**Camera Calibration Parameters:**

The following ROS 2 parameters configure the image compensation and projection modules:

**Image Compensation Parameters:**
```yaml
camera:
  image_compensation_projection:
    ros__parameters:
      camera:
        extrinsic_camera_calibration:
          clip_hist_percent: 1.0
```

**Image Projection Parameters:**
```yaml
camera:
  image_projection:
    ros__parameters:
      camera:
        extrinsic_camera_calibration:
          top_x: 144
          top_y: 41
          bottom_x: 326
          bottom_y: 129
          offset_x: 10
          offset_y: 160
```

**Calibration Interface Usage:**

The calibration interface provides real-time feedback and parameter adjustment capabilities. When calibration mode is enabled, the image_projection node publishes an additional image topic (camera/image_extrinsic_calib) which displays the original camera feed overlaid with red lines marking the trapezoid boundaries. This visual feedback allows operators to see exactly how the region of interest is positioned on the road surface.

![Extrinsic Camera Calibration Interface](turtlebot3_ws/src/maps/projection.png)

This screenshot demonstrates the complete extrinsic camera calibration interface. The central panel shows the live camera feed from the TurtleBot3's perspective, displaying the black track with yellow lane markings. The red trapezoidal overlay clearly marks the region of interest that will be transformed into the bird's eye view. The top-left panel displays the processed output, showing the transformed bird's eye view where the black track surface appears as a flat plane with a prominent yellow lane line visible. The right panel shows the node hierarchy with image_projection highlighted, and below it are the calibration parameter controls. The interface displays all six calibration parameters: top_x (144), top_y (41), bottom_x (326), bottom_y (129), offset_x (10), and offset_y (160), each with sliders or input fields for real-time adjustment. The is_extrinsic_camera_calibration_mode checkbox is checked, indicating calibration mode is active, which enables the red overlay visualization and parameter adjustments.

The right panel of the interface displays all calibration parameters with sliders for interactive adjustment. As parameters are modified in real-time, the red trapezoid overlay updates immediately on the camera feed, and the bird's eye view output in the top-left panel shows the transformed result. This interactive calibration process enables precise alignment of the projection region with the actual road surface.

To perform calibration:
1. Launch the extrinsic calibration with calibration mode enabled
2. Position the robot on the track with lane markings visible
3. Open the rqt_reconfigure GUI to access parameter sliders
4. Adjust top_x, top_y, bottom_x, and bottom_y to align the red trapezoid with the road boundaries
5. Fine-tune offset_x and offset_y to center the trapezoid horizontally and vertically
6. Verify the bird's eye view output shows lane lines as parallel vertical lines
7. Save the calibrated parameters to the projection.yaml file for future use

The calibration parameters are stored in YAML format in the calibration/extrinsic_calibration directory. Once calibrated, these parameters can be loaded automatically on subsequent runs, eliminating the need for repeated calibration unless camera position or orientation changes.

**Image Processing Workflow:**

The complete image processing pipeline flows as follows: Raw camera images are first rectified using intrinsic calibration parameters to remove lens distortion. These rectified images are published to camera/image_rect_color. The image_compensation node enhances brightness and contrast, publishing to camera/image_compensated. The image_projection node then transforms the compensated images into bird's eye view, publishing to camera/image_projected. This final transformed image is used by lane detection algorithms for accurate lane line identification and tracking.

The pipeline supports both compressed and raw image formats, with automatic format detection based on subscribed topics. This flexibility allows the system to operate efficiently in different network conditions, using compressed images to reduce bandwidth when needed.

#### turtlebot3_autorace_detect

This package contains computer vision algorithms for detecting lanes, traffic signs, traffic lights, and level crossings. It processes camera images to extract relevant information for autonomous driving decisions.

**Python Modules:**

**detect_lane.py:** Implements lane detection using color segmentation in HSV color space. The node filters images to extract white and yellow lane markings, identifies lane boundaries, and calculates the lane center position. It uses morphological operations to clean up detected regions and fit lines or curves to the lane markings. The node publishes the detected lane center position as a Float64 message, which is used by the control system to keep the robot centered in the lane.

The detection uses configurable HSV color ranges for white and yellow lane markings, allowing calibration for different lighting conditions and road surfaces.

**Lane Detection Mathematics:**

The lane detection algorithm processes images in the HSV (Hue, Saturation, Value) color space, which is more robust to lighting variations than RGB. The conversion from BGR to HSV is performed using OpenCV's color space conversion. For each pixel with BGR values [B, G, R], the HSV values [H, S, V] are computed as:

```
V = max(R, G, B)
S = (V - min(R, G, B)) / V  if V ≠ 0, else 0

H = {
    60° × (G - B) / (V - min(R, G, B))        if V = R
    60° × (2 + (B - R) / (V - min(R, G, B)))  if V = G
    60° × (4 + (R - G) / (V - min(R, G, B)))  if V = B
}
```

Color thresholding creates a binary mask by checking if each pixel's HSV values fall within specified ranges:

```
mask(x, y) = {
    1  if Hue_l ≤ H(x,y) ≤ Hue_h AND 
          Saturation_l ≤ S(x,y) ≤ Saturation_h AND
          Lightness_l ≤ L(x,y) ≤ Lightness_h
    0  otherwise
}
```

For white lane detection, the ranges are typically:
- Hue: 0 to 179 (full range, since white has no dominant hue)
- Saturation: 0 to 79 (low saturation)
- Lightness: 179 to 255 (high lightness)

For yellow lane detection:
- Hue: 10 to 179
- Saturation: 121 to 255 (high saturation for color visibility)
- Lightness: 95 to 255

The fraction of detected pixels is calculated as:

```
fraction = Σ mask(x, y)  for all pixels
```

Lane detection proceeds only if the fraction exceeds a threshold (typically 3000 pixels).

**Polynomial Lane Fitting:**

Detected lane pixels are fitted to a quadratic polynomial curve. For each lane (left/yellow or right/white), the lane boundary is modeled as:

```
x(y) = a·y² + b·y + c
```

where:
- x: horizontal pixel position
- y: vertical pixel position (from top of image)
- a, b, c: polynomial coefficients (stored as [a, b, c])

The coefficients are computed using least squares polynomial fitting:

```
[a]   [Σy⁴  Σy³  Σy²]⁻¹ [Σx·y²]
[b] = [Σy³  Σy²  Σy ]   [Σx·y ]
[c]   [Σy²  Σy   n  ]   [Σx   ]
```

where the summations are over all detected lane pixels, and n is the number of pixels.

Once fitted, the lane position at any y-coordinate is computed as:

```
x_fit(y) = a·y² + b·y + c
```

For tracking existing lanes, a margin-based search is performed around the previous lane fit:

```
x_min(y) = a_prev·y² + b_prev·y + c_prev - margin
x_max(y) = a_prev·y² + b_prev·y + c_prev + margin
```

Only pixels within this margin are used for the new fit, improving robustness and computational efficiency.

**Moving Average Filtering:**

To smooth lane detection over time, a moving average filter is applied to the polynomial coefficients:

```
a_avg = (1/N) × Σ a_i  for i = 1 to N
b_avg = (1/N) × Σ b_i  for i = 1 to N
c_avg = (1/N) × Σ c_i  for i = 1 to N
```

where N = 5 (MOV_AVG_LENGTH) represents the number of recent frames.

**Lane Center Calculation:**

The lane center position is calculated as the midpoint between the left and right lane boundaries at the bottom of the image (y = image_height):

```
x_left = a_left·y_bottom² + b_left·y_bottom + c_left
x_right = a_right·y_bottom² + b_right·y_bottom + c_right
x_center = (x_left + x_right) / 2
```

This center position is published as a Float64 message on the /detect/lane topic, which is used by the control system to keep the robot centered in the lane.

![Lane Detection Calibration Interface](turtlebot3_ws/src/maps/lane_detect.png)

This image demonstrates the lane detection calibration interface. The central view shows the camera feed with yellow lane markings on a green surface, overlaid with a red trapezoidal region of interest. The right panel displays the HSV color calibration parameters for both white lanes (detect.lane.wh) and yellow lanes (detect.lane.yel), with sliders to adjust hue, saturation, and lightness ranges in real-time. These parameters allow fine-tuning the color detection thresholds to accurately identify lane markings under varying lighting conditions. The top-left panel shows the processed bird's eye view output, displaying the detected lane lines as white vertical lines on a black background, confirming that the lane detection algorithm is working correctly after calibration.

**Commands:**

Launch lane detection:
```bash
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
```

**Lane Detection Parameters:**

The following ROS 2 parameters are used for lane detection based on color thresholding in the **HLS color space**. Both **white** and **yellow** lane markings are detected using configurable hue, lightness, and saturation ranges.

```yaml
/detect_lane:
  ros__parameters:
    detect:
      lane:
        white:
          hue_h: 179
          hue_l: 0
          lightness_h: 255
          lightness_l: 179
          saturation_h: 79
          saturation_l: 0
        yellow:
          hue_h: 179
          hue_l: 10
          lightness_h: 255
          lightness_l: 95
          saturation_h: 255
          saturation_l: 121
```

#### turtlebot3_autorace_mission

This package implements high-level mission control for autonomous racing scenarios. It coordinates between perception, planning, and control systems to execute complex driving behaviors.

**Python Modules:**

**control_lane.py:** Implements a PD (Proportional-Derivative) controller for lane following. The node subscribes to lane center position from the detection system and generates velocity commands to keep the robot centered. It calculates the error between the desired center position and current position, applies proportional and derivative control terms, and publishes Twist commands for linear and angular velocity control.

The controller adjusts linear velocity based on the tracking error, slowing down when the error increases to maintain stability. It includes support for obstacle avoidance mode, where lane following is temporarily suspended in favor of avoidance commands.

**Lane Following Control Mathematics:**

The PD controller calculates the steering command based on the lateral error between the detected lane center and the desired center position. The desired center is fixed at 500 pixels (center of the 1000-pixel wide bird's eye view image).

The position error is computed as:

```
error(t) = x_center(t) - x_desired
```

where:
- x_center(t): detected lane center position at time t
- x_desired: desired center position (500 pixels)

The PD controller calculates the angular velocity command:

```
ω(t) = Kp · error(t) + Kd · (error(t) - error(t-1))
```

where:
- Kp = 0.0025: proportional gain
- Kd = 0.007: derivative gain
- error(t-1): error from previous iteration

The angular velocity is limited to ±2.0 rad/s:

```
ω_limited = {
    max(ω, -2.0)  if ω < 0
    min(ω, 2.0)   if ω ≥ 0
}
```

The linear velocity is adjusted based on the absolute error to slow down when the robot deviates from center:

```
v(t) = min(MAX_VEL · (max(1 - |error(t)|/500, 0))^2.2, 0.05)
```

where:
- MAX_VEL: maximum velocity parameter (typically 0.1 m/s)
- The term (1 - |error|/500)^2.2 provides a smooth velocity reduction as error increases
- Linear velocity is capped at 0.05 m/s for safety

The final velocity commands are published as a Twist message:

```
v_linear = v(t)
ω_angular = -ω_limited(t)
```

The negative sign accounts for the coordinate system convention where positive angular velocity corresponds to left turn, requiring negative angular command to steer right when error is positive.

**Excuted Commands:**

Launch lane control mission:
```bash
ros2 launch turtlebot3_autorace_mission control_lane.launch.py
```

#### turtlebot3_bringup

This package provides launch files and configuration for starting the TurtleBot3 robot system. It includes launch files for bringing up the robot hardware, camera systems, and state publishers. The package handles robot initialization, parameter configuration, and coordinate frame setup.

Key launch files include robot launch for hardware initialization, camera launch for vision system setup, state publisher launch for coordinate transforms, and RViz launch for visualization.

**Excuted Commands:**

Launch the robot with camera:
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Launch the robot state publisher:
```bash
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py
```

Launch RViz for visualization:
```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```


This RViz visualization demonstrates the robot bringup system showing the TurtleBot3 operating within a mapped environment. The central area displays an occupancy grid map where lighter gray regions represent free navigable space and darker gray to black areas indicate obstacles and walls, showing a complex indoor layout with multiple rooms and corridors. The light gray rectangular shape in the center represents the TurtleBot3 robot, with a red dot indicating its forward direction. Numerous thin red lines radiating outward from the robot represent current LiDAR scan readings, showing detected obstacles and the sensor's range boundaries. A dense cluster of small light gray arrows surrounding the robot represents AMCL (Adaptive Monte Carlo Localization) particles, which provide a probabilistic estimate of the robot's pose within the map. The density and spread of these particles indicate the level of localization uncertainty, with tighter clusters indicating higher confidence. Faint red lines extending behind the robot trace its past movement trajectory within the mapped environment. This visualization effectively demonstrates the integration of sensor data, localization, and mapping that occurs when the robot bringup system is running, providing real-time feedback on the robot's position, sensor readings, and environment understanding.

# PART 2 Doing Autonomous Driving by Doing SLAM and Creating the Autorace Map.

### Data Logging Packages

#### tf_logger_pkg

This custom package logs robot pose information from the TF (Transform) tree to CSV files for analysis and visualization.

**Python Module:**

**tf_logger.py:** This code continuously records the robot's position and orientation to a file for later analysis. Think of it as a GPS logger that writes down where the robot is and which direction it's facing.

The code works by listening to the transform tree, which is ROS 2's way of tracking where different parts of the robot are located in space. Specifically, it looks at the relationship between the odom frame (the robot's starting position) and the base_link frame (the robot's current position). This tells the code exactly where the robot has moved to.

Every 0.1 seconds (10 times per second), the code checks the robot's current position. It reads three pieces of information: the X coordinate (how far left or right), the Y coordinate (how far forward or backward), and the yaw angle (which direction the robot is facing, converted from quaternion format to a simple angle).

The code then writes this information to a CSV file called tf_log.csv in your home directory. Each row in the file contains a timestamp, the X position, Y position, and the yaw angle. The timestamp records exactly when each measurement was taken, which allows you to see how the robot moved over time.

This logged data is very useful because you can load it into spreadsheet programs or visualization tools to see the exact path the robot took, analyze its movements, check if it followed the intended route, and evaluate the robot's performance during experiments or missions. The code also handles errors gracefully - if it cannot find the robot's position right away, it will keep trying until the data becomes available.

![Trajectory Visualization on Map](turtlebot3_ws/src/maps/Traject.jpeg)

This image demonstrates the visualization of logged trajectory data. The occupancy grid map shows the explored environment with white areas representing free space, black areas representing obstacles, and gray areas representing unknown regions. The red line overlaid on the map shows the complete trajectory path recorded by the tf_logger node during robot operation. This visualization allows researchers to analyze the robot's movement patterns, verify that it stayed within navigable areas, and identify any irregularities in the path execution.

![Trajectory Path Visualization](turtlebot3_ws/src/maps/Trajectory.png)

This simplified trajectory visualization shows the recorded path in a cleaner format. The white U-shaped line represents the robot's movement path extracted from the logged position data. This type of visualization is useful for quick analysis of path patterns and can help identify loops, turns, and overall navigation behavior.

**Excuted Commands:**

Run the TF logger node:
```bash
ros2 run tf_logger_pkg tf_logger_node
```

The logged data will be saved to ~/tf_log.csv in CSV format with columns: time_sec, x, y, yaw_rad.

### Navigation Packages

#### turtlebot3_cartographer

This package integrates Google Cartographer SLAM algorithm for simultaneous localization and mapping. It includes configuration files for Cartographer, launch files for SLAM execution, and occupancy grid generation. The package enables the robot to build maps of unknown environments while simultaneously tracking its position within those maps.

Launch files provide cartographer SLAM node execution and occupancy grid map generation from SLAM results.

**Excuted Commands:**

Launch Cartographer SLAM:
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
![Robot Localization and Mapping Visualization](turtlebot3_ws/src/maps/rviz.png)


#### turtlebot3_navigation2

Implements autonomous navigation using the Nav2 framework. This package provides path planning, local and global costmap management, behavior tree execution, and recovery behaviors. The navigation stack enables the robot to plan paths to goal locations while avoiding obstacles and following the generated map.

The package includes configuration files for planners, controllers, and behavior trees, along with launch files for starting the complete navigation system.

**Excuted Commands:**

Launch Navigation2 stack:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=$HOME/turtlebot3_ws/src/turtlebot3_autorace/turtlebot3_autorace_mission/map/map.yaml
```

**Note:** Ensure you have a map file generated from SLAM before running navigation.

![Navigation Visualization in RViz](turtlebot3_ws/src/maps/Navigation.png)

This RViz visualization demonstrates the complete navigation system in action. The image shows a top-down view of the TurtleBot3 robot navigating on a mapped track. The light blue area represents free navigable space from the occupancy grid map, while the pink surrounding layer indicates the costmap inflation zone, which encourages the robot to stay centered on the track and avoid obstacles. The gray circular robot is positioned on the track with its forward direction indicated. The dense cluster of green arrows around the robot represents AMCL (Adaptive Monte Carlo Localization) particles, showing the robot's localization uncertainty - each arrow represents a possible pose hypothesis. The concentration of particles indicates good localization confidence. Red dots visible in the scene represent raw LiDAR scan points detecting environmental features and obstacles. The multi-colored trajectory line following the robot's path shows the robot's movement history over time, with colors transitioning from red (most recent) through orange, yellow, green, blue, to purple (oldest), providing a temporal visualization of the navigation path. This visualization effectively demonstrates the integration of SLAM mapping, localization, and navigation components working together for autonomous robot operation.



#### turtlebot3_example

This package provides example implementations demonstrating various TurtleBot3 capabilities and programming patterns.

## Build and Installation

To build this workspace, ensure you have ROS 2 installed and properly sourced. Then execute:

```
cd ~/turtlebot3_ws
colcon build --symlink-install
```

After building, source the workspace:

```
source install/setup.bash
```

Set the TurtleBot3 model environment variable:

```
export TURTLEBOT3_MODEL=burger
```

Or for Waffle model:

```
export TURTLEBOT3_MODEL=waffle
```

## Data and Maps

The workspace includes directories for storing experimental data and generated maps:

**data directory:** Contains logged data files including pose_data.csv with robot trajectory information, centroid_data.csv with detected object positions from vision processing, and JSON formatted data for structured data storage.

**maps directory:** Stores generated occupancy grid maps from SLAM processes, map metadata files, and visualization images showing robot trajectories overlaid on maps.

## Usage

### Camera Calibration

For intrinsic camera calibration, refer to the ROS 2 camera calibration tools. For extrinsic calibration of the projection region, use the image_projection node in calibration mode and adjust parameters using dynamic reconfigure or parameter setting.

### Autonomous Driving

Launch the autonomous driving system using the AutoRace packages. Start the camera calibration nodes, detection nodes, and control nodes. The system will process camera images, detect lanes, and generate velocity commands to follow the detected path.

### SLAM Mapping

Start Cartographer SLAM using the cartographer launch file. Drive the robot around the environment to build a map. The system will publish an occupancy grid map that can be saved using map_saver.

### Navigation

After generating a map with SLAM, use the navigation2 launch file to start the navigation stack. Set navigation goals in RViz or through the navigation action interface. The robot will plan paths and navigate to goals while avoiding obstacles.


## Dependencies

Key ROS 2 packages required:
- rclpy for Python ROS 2 client library
- sensor_msgs for camera and LiDAR message types
- geometry_msgs for pose and twist messages
- nav_msgs for odometry and map messages
- tf2_ros for coordinate transform handling
- cv_bridge for OpenCV and ROS image conversion
- OpenCV for computer vision processing
- Navigation2 stack for path planning and navigation
- Cartographer for SLAM functionality

## Workspace Structure

The workspace follows the standard ROS 2 colcon workspace structure. The workspace root contains the README file, and all source code and build artifacts are organized in standard ROS 2 directories.

### Directory Structure Overview

**src/**: Contains all source packages including:
- TurtleBot3 core packages (bringup, node, description, etc.)
- AutoRace packages for autonomous driving
- Navigation packages (Cartographer SLAM, Navigation2)
- Custom data logging packages
- DynamixelSDK for motor control
- Message definitions

**build/**: Contains build artifacts generated during compilation (created by colcon build)

**install/**: Contains installed packages and setup scripts (created by colcon build)

**log/**: Contains compilation logs and build metadata (created by colcon build)

**src/data/**: Stores experimental data files including:
- Logged trajectory data (CSV, JSON formats)
- Screenshot images from calibration interfaces
- Visualization images

**src/maps/**: Stores generated maps and visualization images:
- Occupancy grid maps from SLAM
- Map metadata files
- Trajectory visualizations
- RViz screenshots

### Source Packages

The workspace contains several categories of packages:

1. **Core TurtleBot3 packages** for robot control and bringup
2. **AutoRace packages** for autonomous racing functionality
3. **Navigation packages** for SLAM and path planning
4. **Data logging packages** for experimental data collection
5. **Example packages** demonstrating various robot capabilities
6. **DynamixelSDK** for motor control and communication

## Workspace Directory Tree

The following tree structure shows the organization of the main workspace directories:

```
turtlebot3_ws/
├── README.md
├── src/
│   ├── data/                          # Experimental data and logged files
│   │   ├── centroid_data.csv
│   │   ├── pose_data.json
│   │   └── tf_log.csv
│   │
│   ├── maps/                          # Generated maps and visualization images
│   │   ├── lane_detect.png
│   │   ├── Lane_detection.webm
│   │   ├── Navigation.mp4
│   │   ├── Navigation.png
│   │   ├── projection.png
│   │   ├── rviz_map.png
│   │   ├── SLAM.webm
│   │   ├── Traject.jpeg
│   │   └── Trajectory.png
│   │
│   ├── tf_logger_pkg/                 # Custom TF logging package
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── tf_logger_pkg/
│   │   │   ├── __init__.py
│   │   │   └── tf_logger.py
│   │   └── test/
│   │
│   ├── turtlebot3/                    # Core TurtleBot3 packages
│   │   ├── turtlebot3/
│   │   ├── turtlebot3_bringup/
│   │   │   ├── launch/                # Launch files
│   │   │   ├── param/                 # Parameter files
│   │   │   └── script/                # Shell scripts
│   │   ├── turtlebot3_node/           # Hardware interface node
│   │   │   ├── include/
│   │   │   ├── src/
│   │   │   └── param/
│   │   ├── turtlebot3_description/    # URDF robot models
│   │   │   ├── meshes/
│   │   │   ├── urdf/
│   │   │   └── rviz/
│   │   ├── turtlebot3_cartographer/   # SLAM package
│   │   │   ├── config/
│   │   │   ├── launch/
│   │   │   └── rviz/
│   │   ├── turtlebot3_navigation2/    # Navigation package
│   │   │   ├── launch/
│   │   │   ├── param/
│   │   │   ├── map/
│   │   │   └── rviz/
│   │   ├── turtlebot3_example/        # Example programs
│   │   │   └── turtlebot3_example/
│   │   └── turtlebot3_teleop/         # Teleoperation
│   │       └── turtlebot3_teleop/
│   │
│   ├── turtlebot3_autorace/           # AutoRace packages
│   │   ├── turtlebot3_autorace/
│   │   ├── turtlebot3_autorace_camera/    # Camera calibration
│   │   │   ├── calibration/
│   │   │   │   └── extrinsic_calibration/
│   │   │   │       ├── compensation.yaml
│   │   │   │       └── projection.yaml
│   │   │   ├── launch/
│   │   │   └── turtlebot3_autorace_camera/
│   │   │       ├── __init__.py
│   │   │       ├── image_compensation.py
│   │   │       └── image_projection.py
│   │   ├── turtlebot3_autorace_detect/    # Detection algorithms
│   │   │   ├── image/                  # Reference images for sign detection
│   │   │   │   ├── construction.png
│   │   │   │   ├── intersection.png
│   │   │   │   ├── left.png
│   │   │   │   ├── parking.png
│   │   │   │   ├── right.png
│   │   │   │   ├── stop.png
│   │   │   │   └── tunnel.png
│   │   │   ├── param/                  # Detection parameters
│   │   │   │   ├── lane/
│   │   │   │   │   └── lane.yaml
│   │   │   │   ├── level/
│   │   │   │   │   └── level.yaml
│   │   │   │   └── traffic_light/
│   │   │   │       └── traffic_light.yaml
│   │   │   ├── launch/                 # Launch files
│   │   │   │   ├── detect_lane.launch.py
│   │   │   │   ├── detect_level_crossing.launch.py
│   │   │   │   ├── detect_sign.launch.py
│   │   │   │   └── detect_traffic_light.launch.py
│   │   │   └── turtlebot3_autorace_detect/
│   │   │       ├── __init__.py
│   │   │       ├── detect_lane.py
│   │   │       ├── detect_traffic_light.py
│   │   │       ├── detect_construction_sign.py
│   │   │       ├── detect_intersection_sign.py
│   │   │       ├── detect_level_crossing_sign.py
│   │   │       ├── detect_level_crossing.py
│   │   │       ├── detect_parking_sign.py
│   │   │       └── detect_tunnel_sign.py
│   │   └── turtlebot3_autorace_mission/   # Mission control
│   │       ├── map/
│   │       ├── param/
│   │       ├── launch/                 # Launch files
│   │       │   ├── control_lane.launch.py
│   │       │   ├── mission_construction.launch.py
│   │       │   └── mission_tunnel.launch.py
│   │       └── turtlebot3_autorace_mission/
│   │           ├── __init__.py
│   │           ├── control_lane.py
│   │           ├── avoid_construction.py
│   │           └── mission_tunnel.py
│   │
│   ├── turtlebot3_msgs/               # Custom message definitions
│   │   ├── action/
│   │   │   └── Patrol.action
│   │   ├── msg/
│   │   │   ├── SensorState.msg
│   │   │   ├── Sound.msg
│   │   │   └── VersionInfo.msg
│   │   └── srv/
│   │       ├── Dqn.srv
│   │       ├── Goal.srv
│   │       └── Sound.srv
│   │
│   └── DynamixelSDK/                  # Dynamixel motor SDK
│       ├── c/                         # C language implementation
│       ├── c++/                       # C++ language implementation
│       ├── c#/                        # C# language implementation
│       ├── python/                    # Python language implementation
│       ├── java/                      # Java language implementation
│       ├── matlab/                    # MATLAB language implementation
│       ├── labview/                   # LabVIEW implementation
│       ├── ros/                       # ROS implementation
│       ├── documents/                 # Documentation
│       ├── LICENSE
│       └── README.md
│
├── build/                             # Build artifacts (generated by colcon build)
├── install/                           # Installed packages (generated by colcon build)
└── log/                               # Build logs (generated by colcon build)
```

**Note:** The `build/`, `install/`, and `log/` directories are automatically generated when running `colcon build` and contain compiled binaries, installed packages, and build logs respectively.

## Project Documentation References

This project was developed following the official TurtleBot3 documentation:

- Camera calibration procedures: ROBOTIS e-Manual TurtleBot3 appendix for Raspberry Pi camera calibration
- Autonomous driving implementation: ROBOTIS e-Manual TurtleBot3 autonomous driving section
- SLAM configuration: ROBOTIS e-Manual TurtleBot3 SLAM node documentation
- Navigation setup: ROBOTIS e-Manual TurtleBot3 navigation documentation
- ChatGPT for Formating mathematics in Readme file



