# QR Code Reader

ROS 2 node for detecting and decoding QR codes from image streams.

## Dependencies

- ROS 2 Jazzy
- OpenCV
- zbar
- cv_bridge
- image_transport
- pyride_common_msgs

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select qrcode_reader
source install/setup.bash
```

## Run

```bash
ros2 run qrcode_reader qrcode_reader
```

## Launch

```bash
ros2 launch qrcode_reader qrcode_reader.launch.py
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera` | string | `/wide_stereo/right/image_rect_color` | Image topic to subscribe to |
| `debug_img` | bool | `false` | Enable debug view of detected images |

## Topics

### Subscribed
- `camera` (sensor_msgs/Image): Raw image stream to process

### Published
- `/qrcode_reader/debug_view` (sensor_msgs/Image): Debug visualization (if `debug_img` is true)
- `/pyride/node_status` (pyride_common_msgs/NodeStatus): Detected QR code data

## Lazy Publishing

This node implements lazy publishing - it only starts processing images when there is an active subscriber. Processing automatically stops when all subscribers disconnect.

## Known Issues

- Requires image stream to be in a format zbar can decode (grayscale or RGBA)