
---

# Image ROS Package

This ROS 2 package, `image_ros`, implements an image conversion node that subscribes to a camera image topic, converts the image to grayscale or leaves it unchanged based on the mode, and republishes the result on another topic. The mode can be toggled dynamically using a ROS 2 service.

---

## Features

- **Image Subscription**: Subscribes to a camera image topic.
- **Image Conversion**: Converts images to grayscale (mode 1) or republishes the original image (mode 2).
- **Dynamic Mode Switching**: Toggle between modes using a ROS 2 service.
- **Image Publication**: Publishes the processed image on an output topic.

---

## Dependencies

Ensure you have the following dependencies installed:

- ROS 2 (Foxy or newer)
- OpenCV (for image processing)
- `cv_bridge` (to bridge OpenCV and ROS 2)
- `image_transport` (for efficient image transport in ROS 2)
- `rqt_image_view` (for image visualization)

Install any missing dependencies:

```bash
sudo apt install ros-foxy-cv-bridge ros-foxy-image-transport ros-foxy-rqt-image-view
```

---

## Installation

### 1. Clone the Package

Clone the `image_ros` package into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/mahachanakya07/image_ros.git
```

### 2. Build the Package

Build the package using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select image_ros
```

### 3. Source the Workspace

Source the workspace to set up the environment:

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Usage

### Running the Nodes

Launch the `image_conversion_node` along with a camera node using the provided launch file:

```bash
ros2 launch image_ros image_conversion_launch.py
```

This will start the image conversion node, which subscribes to the camera topic and republishes the processed image to the output topic.

### Changing the Mode

You can toggle between grayscale and color modes using the ROS 2 service:

- **Grayscale Mode**:

```bash
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"
```

- **Color Mode**:

```bash
ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"
```

### Viewing the Output

You can visualize the output images published on the `/image_converted` topic using `rqt_image_view`:

```bash
rqt_image_view
```

Once `rqt_image_view` is open, select the `/image_converted` topic from the list of available topics. The live feed will be displayed.

---

## Configuration

### Parameters

The node accepts the following parameters:

- **`input_topic`**: The input camera topic (default: `/camera/image_raw`).
- **`output_topic`**: The output topic for converted images (default: `/image_converted`).

You can modify these parameters in the launch file or via the command line.

For example, to use custom topic names:

```bash
ros2 launch image_ros image_conversion_launch.py input_topic:=/my_camera/image_raw output_topic:=/my_converted_image
```

---

## License

This project is licensed under the MIT License.

---
