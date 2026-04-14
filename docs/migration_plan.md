# ROS 1 to ROS 2 Migration Plan for `qrcode_reader`

## Overview

This document outlines the migration plan for converting the `qrcode_reader` package from ROS 1 (Catkin) to ROS 2 (Ament).

**Target Distribution:** ROS 2 Jazzy  
**Package:** `qrcode_reader`  
**Language:** C++  
**Build System:** Catkin → Ament CMake  
**External Dependencies:** OpenCV, zbar, C++11 threading, cv_bridge, image_transport, pyride_common_msgs

---

## Phase 1: Package Manifest Updates

### 1.1 Update `package.xml` (Format 1 → Format 2)

**File:** `package.xml`

| Change | Description |
|--------|-------------|
| Change `<package>` format attribute | `format="1"` → `format="2"` |
| Update buildtool dependencies | Replace `catkin` with `ament_cmake` |
| Update ROS dependencies | `roslib` → `rclcpp`, `rosconsole` → `rclcpp` |
| Update image transport | `image_transport` remains, but version changes |
| Add threading dependency | Replace Boost with C++11 threading |
| Remove `<run_depend>` tags | ROS 2 uses `<exec_depend>` |

**Target Dependencies to Add:**
- `rclcpp`
- `rclcpp_components`
- `ros2_compressed_image_transport` (or standard image_transport)
- `std_msgs` (for Header compatibility)
- `cv_bridge` (ros2 version)
- `image_transport`

---

## Phase 2: Build System Migration

### 2.1 Convert `CMakeLists.txt` (Catkin → Ament CMake)

**File:** `CMakeLists.txt`

| Section | ROS 1 (Catkin) | ROS 2 (Ament) |
|---------|---------------|---------------|
| Build Tool | `cmake_minimum_required(2.8.3)` | `cmake_minimum_required(3.5)` |
| Project | `project(qrcode_reader)` | Same |
| Build Type | Find Catkin | `find_package(ament_cmake REQUIRED)` |
| Dependencies | `catkin_package()` | `ament_target_dependencies()` |
| Include Paths | `include_directories()` | `target_include_directories()` |
| Libraries | `add_library()` / `add_executable()` | Same syntax |
| Install | Handled by Catkin | Explicit `install()` commands required |

**Required `find_package` Components:**
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
```

---

## Phase 3: Header File Migration

### 3.1 Update `include/QRCodeReader.h`

**File:** `include/QRCodeReader.h`

| Line | ROS 1 | ROS 2 Migration |
|------|-------|-----------------|
| 23 | `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| 24 | `#include <image_transport/image_transport.h>` | Same (ROS 2 compatible) |
| 25 | `#include <cv_bridge/cv_bridge.h>` | Same (ROS 2 compatible) |
| 26 | `#include <sensor_msgs/Image.h>` | Same |
| 27 | `#include <sensor_msgs/CompressedImage.h>` | Same |
| 28 | `#include <pyride_common_msgs/NodeStatus.h>` | Same |
| 29 | `#include <boost/thread/mutex.hpp>` | `std::mutex` (C++11) |
| 30 | `#include <boost/thread/recursive_mutex.hpp>` | `std::recursive_mutex` (C++11) |

**Class Structure Changes:**

| ROS 1 Pattern | ROS 2 Equivalent |
|--------------|------------------|
| `ros::NodeHandle nh_` | Node inherits from `rclcpp::Node` (like audio_stream) |
| `ros::Publisher status_pub_` | `rclcpp::Publisher<pyride_common_msgs::NodeStatus>::SharedPtr status_pub_` |
| `ros::Subscriber status_sub_` | `rclcpp::Subscription<pyride_common_msgs::NodeStatus>::SharedPtr` |
| `image_transport::Publisher` | `image_transport::Publisher` (similar API) |
| `image_transport::Subscriber` | `image_transport::Subscription` (similar API) |
| `boost::mutex` | `std::mutex` |
| `boost::recursive_mutex` | `std::recursive_mutex` |
| `CallbackQueue` / `AsyncSpinner` | Remove - use `rclcpp::executors::MultiThreadedExecutor` directly |

---

## Phase 4: Source File Migration

### 4.1 Update `src/QRCodeReader.cpp`

**File:** `src/QRCodeReader.cpp`

#### Initialization (Lines ~38-52)

| ROS 1 | ROS 2 |
|-------|-------|
| `ros::init(argc, argv, "qrcode_reader")` | `rclcpp::init(argc, argv)` |
| `ros::NodeHandle nh` | `auto node = rclcpp::Node::make_shared("qrcode_reader")` |
| `ros::NodeHandle private_nh("~")` | `node->declare_parameter()` pattern |
| `ros::AsyncSpinner spinner(2)` | `rclcpp::executors::MultiThreadedExecutor executor` |
| `spinner.start()` | `executor.add_node(node)` + `executor.spin()` |

#### Parameter Handling (Lines ~48-49)

| ROS 1 | ROS 2 |
|-------|-------|
| `private_nh.param("publish_rate", publish_rate, 10.0)` | `node->declare_parameter("publish_rate", 10.0)` + `node->get_parameter("publish_rate", publish_rate)` |

#### Publisher/Subscriber (Lines ~46-47)

| ROS 1 | ROS 2 |
|-------|-------|
| `status_pub_ = nh.advertise<...>("status", 10)` | `status_pub_ = node->create_publisher<...>("status", 10)` |
| `status_sub_ = nh.subscribe("status", 10, ...)` | `status_sub_ = node->create_subscription<...>("status", 10, ...)` |

#### Image Transport (Lines ~43-44)

| ROS 1 | ROS 2 |
|-------|-------|
| `image_transport::ImageTransport it(nh)` | `image_transport::ImageTransport it(node)` |
| `it.advertiseCamera(...)` | `it.advertiseCamera(...)` (similar API) |

#### Logging (Throughout file)

| ROS 1 | ROS 2 |
|-------|-------|
| `ROS_INFO("...")` | `RCLCPP_INFO(node->get_logger(), "...")` |
| `ROS_ERROR("...")` | `RCLCPP_ERROR(node->get_logger(), "...")` |
| `ROS_WARN("...")` | `RCLCPP_WARN(node->get_logger(), "...")` |

#### Time (Lines ~82, 124)

| ROS 1 | ROS 2 |
|-------|-------|
| `ros::Time::now()` | `node->now()` or capture `auto clock = node->get_clock()` |

#### Rate (Line ~81)

| ROS 1 | ROS 2 |
|-------|-------|
| `ros::Rate rate(publish_rate)` | `rclcpp::Rate rate(publish_rate, node->get_clock())` |

#### Spin (Line ~76)

| ROS 1 | ROS 2 |
|-------|-------|
| `ros::spin()` | `executor.spin()` in MultiThreadedExecutor |

#### Lazy Publishing Implementation (ROS 2 Jazzy Recommended Pattern)

**Reference:** Verified against `/home/xun/ros2_ws/src/pyride` which implements the standard ROS2 node pattern.

ROS 2 provides a `matched_callback` mechanism in `rclcpp::PublisherOptions` that is triggered when subscribers connect/disconnect. This is the **official recommended approach** for ROS 2 Jazzy.

**Implementation:**

```cpp
// In QRCodeReader.cpp - when creating status_pub_
rclcpp::PublisherOptions publisher_options;
publisher_options.event_callbacks.matched_callback =
  [this](const rmw_matched_status_t & status) {
    currentSubscriberCount_.store(status.current_count);
    RCLCPP_INFO(this->get_logger(), "Subscriber event! Current count: %zu",
                currentSubscriberCount_.load());
  };

status_pub_ = create_publisher<pyride_common_msgs::NodeStatus>("status", 10, publisher_options);
```

**Detection Control:**
- Timer-based detection at 10Hz using `create_wall_timer()` instead of custom detection thread
- Detection runs continuously when `currentSubscriberCount_.load() > 0`, idle otherwise
- No need for separate start/stop methods - timer callback checks subscriber count

**Key Components:**
- `rclcpp::PublisherOptions` - Configures publisher event callbacks
- `matched_callback` - Called by ROS 2 when subscribers connect/disconnect
- `rmw_matched_status_t::current_count` - Current number of subscribers
- `std::atomic<size_t>` - Thread-safe storage for subscriber count
- `create_wall_timer()` - Standard ROS2 timer for periodic callbacks

---

## Phase 5: Build Configuration

### 5.1 Library Target

```cmake
add_library(qrcode_reader_lib
  src/QRCodeReader.cpp
)

target_include_directories(qrcode_reader_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(qrcode_reader_lib
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)

ament_target_dependencies(qrcode_reader_lib
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
```

### 5.2 Executable Target

```cmake
add_executable(qrcode_reader src/main.cpp)
target_link_libraries(qrcode_reader qrcode_reader_lib)
ament_target_dependencies(qrcode_reader rclcpp)

install(TARGETS qrcode_reader qrcode_reader_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
```

---

## Phase 6: Launch File Migration

### 6.1 Convert `launch/qrcode_reader.launch`

**File:** `launch/qrcode_reader.launch`

ROS 1 launch files are **not compatible** with ROS 2. Must convert to Python launch files.

**ROS 1 XML → ROS 2 Python:**

```python
# launch/qrcode_reader.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qrcode_reader',
            executable='qrcode_reader',
            name='qrcode_reader',
            output='screen',
            parameters=[{
                'publish_rate': 10.0,
            }]
        ),
    ])
```

---

## Phase 7: Testing

### 7.1 Build Test

```bash
colcon build --packages-select qrcode_reader
```

### 7.2 Source Setup

```bash
source install/setup.bash
```

### 7.3 Run Test

```bash
ros2 run qrcode_reader qrcode_reader
```

---

## File Change Summary

| File | Action |
|------|--------|
| `package.xml` | Rewrite (format 1 → format 2) |
| `CMakeLists.txt` | Rewrite (Catkin → Ament) |
| `include/QRCodeReader.h` | Modify (ROS 1 → ROS 2 API, timer-based detection) |
| `src/QRCodeReader.cpp` | Modify (ROS 1 → ROS 2 API, on_shutdown, matched_callback) |
| `src/main.cpp` | Modify (standard rclcpp::spin() pattern) |
| `launch/qrcode_reader.launch` | Delete (XML) + Create `launch/qrcode_reader.launch.py` |

---

## Migration Sequence

1. **Update `package.xml`** - Set format 2, update dependencies
2. **Rewrite `CMakeLists.txt`** - Ament CMake configuration
3. **Update `include/QRCodeReader.h`** - Replace ROS 1 headers and types
4. **Update `src/QRCodeReader.cpp`** - Migrate all API calls
5. **Update `src/main.cpp`** - Change init pattern
6. **Create `launch/qrcode_reader.launch.py`** - Python launch file
7. **Build and test**

---

## Risk Areas

| Area | Risk | Mitigation |
|------|------|------------|
| Boost dependencies | Some Boost features still used | Replace mutex/threading with C++11 equivalents |
| pyride_common_msgs | Custom message package | Must be available in ROS 2 (verified: pyride uses it) |

---

## Decisions Made

1. **Lazy publishing:** IMPLEMENTED using `matched_callback` pattern from ROS 2 Jazzy
2. **Target distribution:** ROS 2 Jazzy (confirmed by user)
3. **pyride_common_msgs:** Dependency exists in ROS 2 (used by pyride package)
4. **Detection pattern:** Timer-based at 10Hz using `create_wall_timer()` instead of custom thread
5. **Shutdown handling:** Uses `rclcpp::on_shutdown()` for graceful cleanup