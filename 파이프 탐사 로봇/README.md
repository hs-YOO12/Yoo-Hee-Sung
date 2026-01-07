[README.md](https://github.com/user-attachments/files/24466690/README.md)
# In-pipe robot package
In-Pipe Inspection Robot package

# Requirements
1. Ubuntu 22.04 LTS
2. ROS2 Humble
3. RealSense SDK
    - 2.51.1
4. [ipir_6dof_controller](https://github.com/ggonu/ipir_6dof_controller)

> In-Pipe Inspection Robot(IPIR) is a smaller, but robust pipe inspecting robot.

# Usage
## 1) Clone repository
```(bash)
mkdir -p <workspace>/src
cd <workspace>/src
git clone https://github.com/ggonu/inpipe_robot
```

## 2) Build the `project_ipir` package
```(bash)
cd ../
source /opt/ros/humble/setup.bash
```

```(bash)
colbon build
```

## 3) Run Nodes
- Example:
    ```(bash)
        ros2 launch inpipe_robot display.launch.py
    ```
    - You can see the simplicated IPIR URDF and controll the joints to visualize the position of robot.

- **Gazebo Simulation**
  ```(bash)
  ros2 launch inpipe_robot ign.launch.py
  ```
- **IPIR-Controller**
  
- **Segmentation**
  
- **Localization & Mapping**
  

> [!Note]
> Update soon!
