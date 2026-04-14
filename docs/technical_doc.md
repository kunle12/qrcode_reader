# QR Code Reader - Technical Documentation

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                        main.cpp                              │
│  - rclcpp::init() / rclcpp::shutdown()                       │
│  - Creates QRCodeReader node via std::make_shared            │
│  - Calls init() then rclcpp::spin()                          │
│  - rclcpp::on_shutdown() calls fini() automatically          │
└──────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────┐
│                     QRCodeReader (Node)                      │
│  - Inherits from rclcpp::Node                                │
│  - Uses image_transport for camera subscription/publishing   │
│  - Uses matched_callback for lazy publishing                 │
│                                                              │
│  State:                                                      │
│  - imgMsgPtr_: shared ptr to latest image (protected by mutex)│
│  - qrDetectTimer_: wall timer for periodic detection (10Hz)  │
└──────────────────────────────────────────────────────────────┘
                               │
           ┌───────────────────┴───────────────────┐
           ▼                                       ▼
┌─────────────────────┐               ┌─────────────────────────┐
│  Image Subscription │               │   Status Publication    │
│  (image_transport)  │               │   (pyride_common_msgs)  │
│  camera topic       │               │   /pyride/node_status   │
└─────────────────────┘               └─────────────────────────┘
```

## File Structure

```
qrcode_reader/
├── package.xml           # ROS 2 package metadata
├── CMakeLists.txt        # Ament CMake build configuration
├── include/
│   └── QRCodeReader.h    # Node class definition
├── src/
│   ├── QRCodeReader.cpp  # Implementation
│   └── main.cpp          # Entry point
├── launch/
│   └── qrcode_reader.launch.py  # Launch configuration
└── docs/
    ├── migration_plan.md      # ROS 1 → ROS 2 migration notes
    └── technical_doc.md       # This document
```

## Key Implementation Details

### Lazy Publishing Pattern

The node uses ROS 2's `matched_callback` mechanism to implement lazy publishing (start/stop based on subscriber connections):

```cpp
rclcpp::PublisherOptions publisher_options;
publisher_options.event_callbacks.matched_callback =
  [this]( const rmw_matched_status_t & status ) {
    RCLCPP_INFO( this->get_logger(), "Subscriber event! Current count: %zu",
                 status.current_count );
    if (status.current_count > 0) {
      startDetection();
    } else {
      stopDetection();
    }
  };

status_pub_ = this->create_publisher<pyride_common_msgs::msg::NodeStatus>(
  "/pyride/node_status", 1, publisher_options );
```

### Image Transport

- Uses `image_transport::ImageTransport` initialized with `shared_from_this()`
- Subscriber callback `processingRawImages` stores image pointer
- Timer callback `doDetection()` runs zbar detection at 10Hz (kDetectionRateMs = 100)

### Thread Safety

- `std::mutex` protects `imgMsgPtr_` access between subscriber callback and timer callback
- Detection timer canceled in `stopDetection()` before shutdown
- `rclcpp::on_shutdown()` ensures cleanup on node shutdown

### Shutdown Handling

The node registers an `rclcpp::on_shutdown()` callback in the constructor to ensure proper cleanup:

```cpp
QRCodeReader::QRCodeReader()
  : Node( "qrcode_reader" ),
    imgTrans_( shared_from_this() ),
    showResult_( false )
{
  zbarScanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  rclcpp::on_shutdown( [this]() { this->fini(); } );
}
```

## ROS 2 API Reference

| ROS 1 | ROS 2 |
|-------|-------|
| `ros::init()` | `rclcpp::init()` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `ros::NodeHandle` | Inherit from `rclcpp::Node` |
| `image_transport::ImageTransport(*nh)` | `image_transport::ImageTransport(shared_from_this())` |
| `sensor_msgs::ImageConstPtr` | `sensor_msgs::msg::Image::ConstSharedPtr` |
| `boost::mutex` | `std::mutex` |
| `boost::thread` | Removed (use `create_wall_timer` instead) |

## Include Paths (ROS 2)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <pyride_common_msgs/msg/node_status.hpp>
```

## Message Types

- **Subscribed**: `sensor_msgs/msg/Image`
- **Published**: `pyride_common_msgs/msg/NodeStatus`
  - `header.stamp`: `this->now()`
  - `priority`: 2
  - `for_console`: false
  - `node_id`: "qrcode_reader"
  - `status_text`: semicolon-separated QR code data

## cv_bridge Encoding

Uses string encoding `"mono8"` instead of `sensor_msgs::image_encodings::MONO8`:

```cpp
cv_ptr = cv_bridge::toCvCopy( imgMsgPtr_, "mono8" );
```

## Debugging

Enable debug view with `debug_img` parameter - publishes to `/qrcode_reader/debug_view`.

RCLCPP_INFO is used for logging:
- Subscriber count changes
- Detected barcode data
- Detection start/stop events

## Testing

```bash
# Build
colcon build --packages-select qrcode_reader

# Source and run
source install/setup.bash
ros2 run qrcode_reader qrcode_reader
```

## Common Issues

1. **Image format**: zbar requires grayscale (mono8) or RGBA images
2. **Topic mismatch**: Ensure `camera` parameter matches actual image topic
3. **Thread safety**: Never access `imgMsgPtr_` without locking the mutex