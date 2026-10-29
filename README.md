# T265-demo

A small Qt 6 viewer for the Intel RealSense T265. Shows the 6-DoF
trajectory, both fisheye images and the gyro/accelerometer strips, with a
live readout (position, orientation, per-stream Hz, tracker confidence) and
CSV export of the pose buffer.

The camera is read either

* directly through the librealsense C++ SDK (`--mode sdk`, default), or
* through ROS 2 topics published by the `realsense2_camera` wrapper node
  (`--mode ros`).

Both backends implement the same `ISensorSource` interface, so the GUI is
identical in either mode.

## Features

* 3-D trajectory canvas (drag to rotate, wheel to zoom)
* live fisheye 1 / fisheye 2 (848x800 @ 30 fps, Y8)
* gyro @ 200 Hz and accelerometer @ 62 Hz strip plots
* readout: position, yaw/pitch/roll, per-stream Hz, tracker confidence
* CSV export: `t,x,y,z,qx,qy,qz,qw` (epoch seconds, quaternion)
* clean status hints when no device / no topics are available

## Build

Requirements: Qt 6 (Widgets, OpenGL), CMake >= 3.22, C++20 compiler.
For SDK mode: the librealsense development files (the T265 port adds the
tracking headers, see the `librealsense-t265-v2.59.1` kit).
For ROS mode: a ROS 2 installation (tested on Lyrical); the CMake script
detects `rclcpp` and builds the ROS backend only when it is present.

```
cmake -B build            # SDK-only build
cmake -B build-ros        # inside a sourced ROS 2 env: SDK + ROS
cmake --build build-ros
```

## Usage

```
t265-demo [--mode sdk|ros] [--topic-prefix PREFIX]
          [--dump-shot FILE] [--dump-csv FILE] [--dump-delay SECS]
```

* `--mode sdk` — talk to the camera directly via librealsense (default).
* `--mode ros` — subscribe to the wrapper's topics:
  `<prefix>/pose/sample` (nav_msgs/Odometry),
  `<prefix>/fisheye{1,2}/image_rect_raw` (sensor_msgs/Image, mono8),
  `<prefix>/gyro/sample`, `<prefix>/accel/sample`
  (sensor_msgs/Imu). Default prefix `/camera/camera`.
* `--topic-prefix P` — change the topic prefix (default `/camera/camera`).
* `--dump-shot FILE` / `--dump-csv FILE` / `--dump-delay SEC` — headless
  self-dump: after the delay (default 25 s) save the window as PNG and/or
  the pose buffer as CSV, print the per-stream frame counts, and quit.
  Useful for automated checks without a display.

### SDK mode

Just run it. If the T265 is plugged in (USB 3.0), the app opens
tracking @ 200 Hz, both fisheyes @ 30 fps, gyro @ 200 Hz and accel @
62 Hz.

### ROS mode

Start the wrapper node first, then run the app:

```
ros2 launch realsense2_camera rs_launch.py   # wrapper (separate terminal)
t265-demo --mode ros
```

The app shows a "waiting for topics" hint until the node is up. Note that
the wrapper's default parameters enable only pose + the two fisheye
streams; gyro/accel topics are only published if `enable_gyro` /
`enable_accel` are set to true.

## Notes

* The T265 enters bootloader mode on plug-in; librealsense uploads the
  firmware automatically on first `query_devices()` (the app's device
  search loop waits for this, up to ~45 s).
* The device is single-holder: the SDK app and the wrapper node cannot use
  it at the same time.
* The wrapper publishes sensor topics with best-effort QoS, so the app
  subscribes with `rclcpp::SensorDataQoS()` — a reliable subscription would
  silently receive nothing.

## Layout

```
src/
  main.cpp             CLI, app wiring, self-dump
  source.h/.cpp        ISensorSource interface (Qt signals)
  sdk_source.*         librealsense backend (composite Tracking Module sensor)
  ros_source.*         ROS 2 backend (rclcpp subscriptions)
  gui_window.*         main window, readout, CSV export
  trajectory_canvas.*  3-D trajectory viewport
  imu_strip.*          gyro/accel strip plots
```

## License

Apache License Version 2.0, January 2004 http://www.apache.org/licenses/
