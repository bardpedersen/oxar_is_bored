# Xbox Controller Button Mapping

This README file provides a visual mapping of every button on an Xbox controller and its function.

![Xbox Controller](x_box_controller_diagram.png)

## Button Descriptions

1. **Rigth Stick**: Moves arm(s) left and right
2. **D-pad**: Controlles the end effectors
3. **Left stick**: Moves arm(s) forward backwards
4. **Back Button**: Decreese the arm speed
5. **LB Button**: Activate/deactivate left arm
6. **LT Button**: Controll the arm in z-direction, LT is positiv direction
7. **Xbox Button**: Moves the arm two end position
8. **Start Button**: Increese the arm speed
9. **RT Button**: Controll the arm in z-direction, RT is negativ direction
10. **RB Button**: Activate/deactivate right arm
11. **Y Button**: Activate/deactivate endeffector on right arm
12. **B Button**: Moves the arm and end effectors to home position when they are active
13. **A Button**: Switches between global and local frame of controll
14. **X Button**: Activate/deactivate endeffector on left arm
15. **R3**: Enables and deables the mirror function

**L3** and **R3** the safety service is called and the robot stops.

**LT** and **RT** need to be pressed down completly before they are in use.

## Usage

You can refer to this button mapping diagram when driving thorvald and controlling the arms with the teleop_arm.py

When endeffectors are not active they will follow the arms so that the camera always follows the y-axis (global frame). The tilt will remain from its last setpoint.

**L3** is **Left stick** pressed down, and **R3** is **Right stick** pressed down.
