---
trigger: glob
globs: src/third_party/go2_ros2_sdk/**/*.py
description: Python rules for go2_ros2_sdk edits (minimal changes)
labels: python,third_party
---

# go2_ros2_sdk (Python) Rules

- Keep changes minimal and focused.
- Avoid introducing new heavy dependencies unless required.
- Guard platform-specific imports.
- Prefer logging over prints where applicable.
