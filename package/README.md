# quadrotor_motion_with_mpc_control
## behavior_follow_path_with_mpc_control

**Purpose**: This behavior follows a path while using the path tracker. This behavior does not finish until the robot reach the last point. The argument is a tuple of coordinates.

**Type of behavior:** Goal-based behavior.

**Arguments:** 

| Name    |   Format  |  Example |  
| :-----------| :---------| :--------|
| path: |Tuple of coordinates x, y, z (meters)|path: [[1.23, 2, 0.5],[2.5,3,1],[3.1,3.4,1.5]]|  
| yaw: |Allowed values {constant, path_facing}|yaw: "constant"| 

----
## behavior_hover_with_mpc_control

**Purpose:** The robot keeps hovering. Hovering is a maneuver in which the robot is maintained in nearly motionless flight over a reference point at a constant altitude and on a constant heading. This behavior does not avoid moving obstacles. 

**Type of behavior:** Recurrent behavior.

----
## behavior_rotate_with_mpc_control

**Purpose:** The robot rotates left or right a certain number of degrees (angle) on the vertical axis (yaw). The number of degrees can be expressed as an absolute value (argument “angle”) or relative to the robot (argument “relative_angle”).

**Type of behavior:** Goal-based behavior.

| Arguments    |   Format  |  Example |  
| :-----------| :---------| :--------|          
| angle |number (degrees)|angle: 90|
| relative_angle |number (degrees)|relative_angle: 90|


# Contributors

**Code maintainer:** Alberto Rodelgo Perales

**Authors:** Alberto Rodelgo Perales