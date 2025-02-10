# PX100 Pointing Controller Package

This package controls the Interbotix **PX100** robotic arm to move its end-effector to a specified target position. If the target is out of reach, the position is adjusted to fit within the robot's maximum reach while maintaining the direction.

---

## **Installation**

Before running the package, make sure you have built the workspace using:

```bash
colcon build --symlink-install
```

This command builds the workspace with symbolic links to the Python files, allowing you to modify the code without rebuilding the package.

---

## **Running the Simulation**

1. **Launch the Interbotix control interface:**  
   In a terminal, execute the following command to launch the simulation interface for the **mobile_px100** robot:

   ```bash
   ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=mobile_px100 use_sim:=true
   ```

   This will launch a graphical interface where you can visualize and interact with the simulated PX100 robot.

2. **Run the Pointing Controller node:**  
   In a new terminal, run the pointing controller node with the following command:

   ```bash
   ros2 run px100_pointing_controller_py px100_pointing_node
   ```

---

## **Code Overview**

### **Main Classes and Methods**

- **`PX100PointingNode(Node)`**:  
  Initializes the Interbotix PX100 robot and controls the arm's movements.

  - **Method: `adjust_target_to_reach_limit(x, y, z)`**  
    - Description: This method verifies if the target position is within the robot's maximum reach. If the target is too far, it scales the position down to fit within the reach limit.
  
  - **Method: `point_to_target(x, y, z, yaw=0.0)`**  
    - Description: Moves the robot's end-effector to the specified target position. If necessary, the target is adjusted to a reachable position.

---

## **Example Usage**

The controller node allows you to specify a target pose for the robot's end-effector. By default, the code in `pointing_controller.py` tries to reach a target at `(x=0.4, y=0.4, z=0.5)`:

```python
node.point_to_target(0.4, 0.4, 0.5, yaw=0.0)
```

If this point is too far for the robot to reach, it is adjusted according to the maximum reach limit (`0.28 meters` in the code).

You can modify the target position by changing the values in the `point_to_target` method call:

```python
node.point_to_target(-0.05, 0.10, 0.15, yaw=0.0)
```

---

## **Dependencies**

Make sure you have the following dependencies installed:

- **ROS 2 Humble** (or your specific ROS 2 distribution)
- **Interbotix ROS Packages** for XS-series manipulators
- **Python 3**

---

## **Troubleshooting**

- If the robot does not move to the target position, verify that the target is within the maximum reach. You can reduce the `MAX_REACH` value if needed.
- Ensure that the simulation is running correctly with the appropriate model (`mobile_px100`).

---

With these instructions, you should be able to run the package successfully and control the PX100 robot!
