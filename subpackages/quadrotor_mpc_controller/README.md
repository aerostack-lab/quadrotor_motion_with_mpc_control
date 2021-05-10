# Quadrotor MPC Controller

Control strategies for rotary wing Micro Aerial Vehicles (MAVs) using ROS. Original code: [https://github.com/ethz-asl/mav_control_rw.git](https://github.com/ethz-asl/mav_control_rw.git)

## Overview

This repository contains a Linear MPC for MAV trajectory tracking.

Moreover, an external disturbance observer based on Kalman Filter is implemented to achieve offset-free tracking. 

If you use any of these controllers within your research, please cite one of the following references

```bibtex
@incollection{kamelmpc2016,
                author      = "Mina Kamel and Thomas Stastny and Kostas Alexis and Roland Siegwart",
                title       = "Model Predictive Control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System",
                editor      = "Anis Koubaa",
                booktitle   = "Robot Operating System (ROS) The Complete Reference, Volume 2",
                publisher   = "Springer",
                year = “2017”,
}
```

```bibtex
@ARTICLE{2016arXiv161109240K,
          author = {{Kamel}, M. and {Burri}, M. and {Siegwart}, R.},
          title = "{Linear vs Nonlinear MPC for Trajectory Tracking Applied to Rotary Wing Micro Aerial Vehicles}",
          journal = {ArXiv e-prints},
          archivePrefix = "arXiv",
          eprint = {1611.09240},
          primaryClass = "cs.RO",
          keywords = {Computer Science - Robotics},
          year = 2016,
          month = nov
}

```

## Supported autopilots

### Asctec Research Platforms
This control will work as is with the ros interface to the now discontinued Asctec research platforms (Hummingbird, Pelican, Firefly and Neo). 

### Pixhawk
This controller requires some small modifications to the PX4 firmware to allow yaw rate inputs. A modified version of the firmware can be found [here](https://github.com/ethz-asl/ethzasl_mav_px4). The firmware is interfaced with through a [modified mavros node](https://github.com/ethz-asl/mavros).

### DJI
The controller can interface with DJI platforms through our [mav_dji_ros_interface](https://github.com/ethz-asl/mav_dji_ros_interface)

## Subscribed topics

- **motion_reference/trajectory** ([trajectory_msgs/MultiDOFJointTrajectory](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html))  
This is a desired trajectory reference that includes desired velocities and accelerations.

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
This is a reference set point.

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle.

- **self_localization/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))     
Current speed of the vehicle.

## Published topics

- **actuator_command/roll_pitch_yaw_rate_thrust** ([mav_msgs/RollPitchYawrateThrust](http://docs.ros.org/api/mav_msgs/html/msg/RollPitchYawrateThrust.html))           
Actuator command for the multirotor specifying roll (rad), pitch (rad), yaw rate (rad/s) and thrust (N: Newtons).

- **motion_reference/remaining_path** ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))           
Remaining points of the path being followed by the multirotor. This list of points is updated each time a point is reached. When the remaining path is updated with an empty list of points, the reference path has been completed by the multirotor. Each point is defined with a pose <x, y, z, yaw> (the values for <pitch, roll> are discarded).

- **motion_reference/assumed_pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))     
Pose reference assumed by the controller.

## Parameters
A summary of the linear and nonlinear MPC parameters:

| Parameter             | Description                                                                     |
| --------------------  |:-------------------------------------------------------------------------------:| 
| `use_rc_teleop`       | enable RC teleoperation. Set to `false` in case of simulation.                  |
| `reference_frame`     | the name of the reference frame.                                                |
| `verbose`             | controller prints on screen debugging information and computation time          |
| `mass`                | vehicle mass                                                                    | 
| `roll_time_constant`  | time constant of roll first order model                                         |
| `pitch_time_constant` | time constant of pitch first order model                                        |
|`roll_gain`            | gain of roll first order model                                                  |
|`pitch_gain`           | gain of pitch first order model                                                 |
|`drag_coefficients`    | drag on `x,y,z` axes                                                            |
|`q_x, q_y, q_z`*       | penalty on position error                                                       |
|`q_vx, q_vy, q_vz`*    | penalty on velocity error                                                       |
|`q_roll, q_pitch`*     | penalty on attitude state                                                       |
|`r_roll, r_pitch, r_thtust`*| penalty on control input                                                    |
|`r_droll, r_dpitch, r_dthtust`*| penalty on delta control input (only Linear MPC)                        |
|`roll_max, pitch_max, yaw_rate_max`*| limits of control input                                             |
|`thrust_min, thrust_max`* | limit on thrust control input in `m/s^2`                                      |
|`K_yaw`*                  | yaw P loop gain                                                               |
|`Ki_xy, Ki_z`*            | integrator gains on `xy` and `z` axes respectively                            |
|`position_error_integration_limit` | limit of position error integration                                 |
|`antiwindup_ball`        | if the error is larger than this ball, no integral action is applied          |
|`enable_offset_free`*     | use estimated disturbances to achieve offset free tracking                    |
|`enable_integrator`*      | use error integration to achieve offset free tracking                         |
|`sampling_time`          | the controller sampling time (must be equal to the rate of `odometry` message |
|`prediction_sampling_time`| the prediction sampling time inside the controller                           |


\* Through dynamic reconfigure, it is possible to change these parameters.

## Implementation for Aerostack

Some changes have been made to the original code in order to integrate this node in Aerostack. These changes are wrapped by comments with the word 'AEROSTACK' in the current code, however the details are described below:

| File                                                     | Changes                                                                                                                                     |
|----------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| mav_control_interface/src/mav_control_interface_impl.cpp | Changed topics, added start and stop services, changed odometry callback to pose and speed callbacks                                        |
| mav_control_interface/src/mpc_queue.cpp                  | Added publishers for remaining path and assumed pose. Added setRemainingPath(), checkPathReferences() and publishPathReferences() functions |
| mav_control_interface/src/state_machine.cpp              | Changed topics                                                                                                                              |
| mav_linear_mpc/src/linear_mpc.cpp                        | Calls to functions setRemainingPath() and checkPathReferences()                                                                             |

## References

[1] Model Predictive Control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System. Mina Kamel, Thomas Stastny, Kostas Alexis and Roland Siegwart. Robot Operating System (ROS) The Complete Reference Volume 2. Springer 2017 (to appear)

[2] Linear vs Nonlinear MPC for Trajectory Tracking Applied to Rotary Wing Micro Aerial Vehicles. Mina Kamel, Michael Burri and Roland Siegwart. arXiv:1611.09240

## Contributors

**Author:** Mina Kamel fmina(at)ethz.ch
**Code maintainer:** Alberto Rodelgo