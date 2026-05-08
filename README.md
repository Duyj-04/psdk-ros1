# psdk-ros1

`psdk_ros1` is a ROS Noetic wrapper for DJI Payload SDK 3.13.

## Layout expectation

This package is designed as a **single catkin package** and links directly against a local SDK checkout at:

```text
../Payload-SDK-3.13.0
```

Place `psdk-ros1` and `Payload-SDK-3.13.0` as sibling directories before building.

## Build

```bash
catkin_make
```
