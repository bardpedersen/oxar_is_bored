# Xbox Controller Button Mapping for arms and wheel

This README file provides a visual mapping of every button on an Xbox controller and its function.

![Xbox Controller](x_box_controller_diagram.png)

## Button Descriptions

1. **Rigth Stick**: Controll Thorvald
2. **D-pad**: Controlling end effectors
3. **Left stick**: Controlling arms
4. **Back Button**: Used to decreese the arm speed
5. **LB Button**: Decreese the driving speed
6. **LT Button**: Controll the arm in z-direction, LT is positiv direction
7. **Xbox Button**: -----
8. **Start Button**: Increese the arm speed
9. **RT Button**: Controll the arm in z-direction, RT is negativ direction
10. **RB Button**: Increese the driving speed
11. **Y Button**: Switches between the end effectors
12. **B Button**: Moves the arm to home position, moves the end effector to home position when Y is beeing pressed.
13. **A Button**: Call the homing service that realign the wheels
14. **X Button**: Switches between the arms

**L3** and **R3** the safety service is called and the robot stops.

## More info

**LT** and **RT** need to be pressed down completly before they are in use.

**L3** is **Left stick** pressed down, and **R3** is **Right stick** pressed down.

## Usage
You can refer to this button mapping diagram when driving thorvald and controlling the arms with the teleop_arm_and_wheels.py

When endeffectors are not active they will follow the arms so that the camera always follows the y-axis (global frame). The tilt will remain from its last setpoint.