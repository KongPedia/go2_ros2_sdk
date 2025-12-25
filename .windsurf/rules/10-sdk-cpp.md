---
trigger: glob
globs: src/third_party/go2_ros2_sdk/**/*.cpp,src/third_party/go2_ros2_sdk/**/*.hpp,src/third_party/go2_ros2_sdk/**/*.h,src/third_party/go2_ros2_sdk/**/CMakeLists.txt,src/third_party/go2_ros2_sdk/**/package.xml
description: C++/ROS 2 rules for go2_ros2_sdk edits
labels: cplusplus,ros2,rclcpp
---

# go2_ros2_sdk (C++ / ROS 2) Rules

- Use C++17-compatible code.
- Prefer clear ownership semantics (RAII) and avoid raw `new/delete`.
- Keep node execution non-blocking; avoid long blocking loops in callbacks.
- If adding nodes, prefer ROS 2 best practices (parameters, QoS explicitness).
- Keep build system changes minimal and localized.
