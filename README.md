# Destroyer Workspace

## Purpose
This workspace is designed for the implementation and testing of **GridSLAM** (Grid-based Simultaneous Localization and Mapping) and a **State Machine** for autonomous navigation.

- **GridSLAM**: Uses a grid-based approach to map the environment while simultaneously tracking the robot's pose. It handles sensor data integration, occupancy grid updates, and pose estimation.
- **State Machine**: Given an initial pose and a goal pose, it finds a path and follows it automatically using the generated map.

## GridSLAM Implementation
The core of this workspace is the GridSLAM implementation. It allows you to create a map of the environment using a particle filter for localization and an occupancy grid for mapping.

## Running the Simulation and SLAM

Execute each `ros2 launch` or `ros2 run` command in a separate terminal window. Follow the steps below to build the workspace and launch the necessary components.

### 1. Build the Workspace
First, build all packages in the workspace:
```bash
colcon build
```

### 2. Launch GridSLAM (includes Odometry)
In a new terminal, source the workspace and launch the GridSLAM node.
```bash
source install/setup.bash
ros2 launch grid_fastslam fastslam.launch.py num_particles:=50
```

### 3. Launch Custom Casa Simulation
In a new terminal, source the workspace and launch the custom simulation environment:
```bash
source install/setup.bash
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
```

### 4. Run Teleoperation
In a new terminal, run the teleoperation node to control the robot and explore the environment to build the map:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### 5. Save the Map
Once you are satisfied with the map, save it. The map saver node will save the map to the `src/state_machine/maps` directory.
```bash
source install/setup.bash
ros2 run tools map_saver --ros-args -p map_name:=my_map
```

---

## Running the State Machine

After creating and saving a map, you can run the State Machine to navigate autonomously.

### 1. Build the Workspace
Since a new map was added, rebuild the workspace to ensure it is installed correctly:
```bash
colcon build
```

### 2. Launch State Machine
In a new terminal, source the workspace and launch the State Machine. This will launch the map publisher, localization, and navigator nodes.
```bash
source install/setup.bash
ros2 launch state_machine state_machine.launch.py
```

### 2. Launch Environment
If the simulation is not already running, launch it in a new terminal:
```bash
source install/setup.bash
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
```

### 3. Set Initial Pose and Goal in Rviz
1.  **Initial Pose**: Use the "2D Pose Estimate" tool in Rviz to set the initial pose of the robot to match its position in the simulation.
2.  **Goal Pose**: Use the "2D Goal Pose" tool in Rviz to set a destination. The State Machine will calculate a path and navigate the robot to the goal.