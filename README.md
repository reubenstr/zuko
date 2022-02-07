# Zuko - A quadruped robot dog!

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v2/zuko-robot-dog-cad-design.png" width="640"> | 
------------ | 
CAD rendering of v2 design. |

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v1/zuko-australian-shepard-mini.jpg" width="640"> | 
------------ | 
Zuko himself! |

<img src="https://github.com/reubenstr/Zuko/blob/main/diagrams/linked-leg-kinematic-dragram.png" width="640"> | 
------------ | 
Linked leg kinematic diagram. |

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v1/zuko-v1-first-standing-on-power.jpg" width="640"> | 
------------ | 
v1 First time standing. |


Status: **Development.**   


Major Tasks To Do:
- Assemble v2 frame and electronics.
- Create URDF model for simulation.
- Refine motor limits.
- Code BN055 IMU.
- Connect machine learning algorithms.

ROS2 Workspace (contains quadruped source code): 

https://github.com/reubenstr/quad_ws



My contributions:
- Migration of source code from ROS1 to ROS2 (Robot Operating System) frameworks.
- Inverse kinematics for a linked leg system for OpenQuadruped / Spot Mini Mini based quadrupeds.
- Parameterized duplicated hard-coded values.
- Created servo calibration script for quicker calibration and servo support.
- A frame redesign using larger servos, providing more space for electronics, rework of the leg/hip system, and addition of aesthetic features (tail, etc).
- A more feature rich motor and peripheral expansion board using a Teensy 4.0 supporting aux servos, foot sensors, NeoPixls, etc.
- 8x 18650 user accessable battery holder with onboard BMS (battery management system). 
 
Credits:
- Simulation, kinematics, bezier curves, and machine learning are largely based on the work from Spot Mini Mini: https://github.com/OpenQuadruped/spot_mini_mini 
- Spot Mini Mini is based on SpotMicroAI: https://spotmicroai.readthedocs.io/en/latest/
- Frame and leg linkage design inspired by Baris Alp's quadruped design: https://grabcad.com/library/diy-quadruped-robot-1
