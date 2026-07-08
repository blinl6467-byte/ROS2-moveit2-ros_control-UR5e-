# ur_description

UR5e-only robot description package for this workspace.

This package provides:

- URDF/Xacro model files
- UR5e visual and collision meshes
- UR5e kinematics, joint-limit, physical, and visual parameters
- RViz model-view launch files

The model entry point is:

```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
```

In this project the public launch entry point is:

```bash
ros2 launch ur5e_bringup view_model.launch.py
```
