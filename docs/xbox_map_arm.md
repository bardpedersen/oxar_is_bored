# Xbox Controller Button Mapping for arms

This README file provides a visual mapping of every button on an Xbox controller and its function.

![Xbox Controller](x_box_controller_diagram.png)

## Button Descriptions

1. **Rigth Stick**: Moves arm(s) left and right
    1. **R3**: Enables and deables the mirror functio
2. **D-pad**: Controlles the end effectors
3. **Left stick**: Moves arm(s) forward backwards
    1. **L3**: Switches between the functions of xbox button. Functions: Free, Calibration, Away, Move preset, Move sinusodial, Move cosinusodial
4. **Back Button**: Decreese the arm speed
5. **LB Button**: Activate/deactivate left arm (2)
6. **LT Button**: Controll the arm in z-direction, LT is positiv direction
7. **Xbox Button**: Activates the choosen fuction. The functions are choosen by L3.
8. **Start Button**: Increese the arm speed
9.  **RT Button**: Controll the arm in z-direction, RT is negativ direction
10. **RB Button**: Activate/deactivate right arm (1)
11. **Y Button**: Activate/deactivate endeffector on right arm (1)
12. **B Button**: Moves the arm and end effectors to home position when they are active
13. **A Button**: Switches between global and local frame of controll
14. **X Button**: Activate/deactivate endeffector on left arm (2)

**L3** and **R3** the safety service is called and the robot stops.

## More info
**LT** and **RT** need to be pressed down completly before they are in use.

**L3** is **Left stick** pressed down, and **R3** is **Right stick** pressed down.

The different functions for the xbox button:
1. Free: The arm(s) can be moved freely, no need to press xbox button to activate.
2. Calibration: The arm(s) will calibrate
3. Away: The arm(s) will move to away position, along the x-axis
4. Move preset: The arm(s) will move to the points in the premove list from the config file
5. Move sinusodial: The arm(s) will move in a sinusodial motion 
6. Move cosinusodial: The arm(s) will move in a cosinusodial motion

Only the activated arm(s) will move


## Usage

You can refer to this button mapping diagram when driving thorvald and controlling the arms with the teleop_arm.py

When endeffectors are not active they will follow the arms so that the camera always follows the y-axis (global frame). The tilt will remain from its last setpoint.
