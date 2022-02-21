# Zuko - A quadruped robot dog!

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v2.1/cad/zuko-v2.1-cad-rendering-side.png" width="640"> | 
------------ | 
Frame v2.1 CAD design. |

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v1/zuko-australian-shepard-mini.jpg" width="640"> | 
------------ | 
Zuko himself! |

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v2.1/assembling/zuko-robot-dog-printed-parts-exploded-v2.1.jpg" width="640"> | 
------------ | 
Frame v2.1 parts printed. |

<img src="https://github.com/reubenstr/Zuko/blob/main/pcbs/expansion-board/expansion-board-prototype-render.png" width="640"> | 
------------ | 
Expansion board v1 for direct RPi inteface. |

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v2.1/testing/zuko-v2.1-first-time-standing-2.jpg" width="640"> | 
------------ | 
Zuko frame v2.1 first time standing. |

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v2.1/assembling/zuko-v2.1-linked-leg-bridge-bearing-assembly-front.jpg" width="640"> | 
------------ | 
Frame v2.1 upper leg, linked leg bridge, and servo horn interfaces press fit together. |

### Status

**Development.**  

**V2.1 Frame is being tested and is nearly ready for replication**

### General

Zuko uses the ROS2 framework for communication, launching, and 3rd party addons. The ROS2 workspace contains the quadruped source code and operates inside Ubuntu either on a PC or RPi. The motors are controlled by a Teensy 4.0 microcontroller on a separate board which is connected to either the PC or RPi over USB or UART.

### Major Tasks To Do-
- Fab/test expansion-board (RPi direct control, no Teensy)
- Create URDF model for simulation.
- Refine motor limits, update kinematic model to frame v2.1.
- Calibrate walking parameters.
- Code BN055 IMU.
- Connect machine learning algorithms.

### ROS2 Workspace 

Quadruped source code: https://github.com/reubenstr/quad_ws

### My contributions
- Frame redesign using larger servos, providing more space for electronics, rework of the leg/hip system, and addition of aesthetic features (tail, etc).
- Migration of source code from ROS1 to ROS2 (Robot Operating System) frameworks.
- Inverse kinematics for a linked leg system for OpenQuadruped / Spot Mini Mini based quadrupeds.
- Parameterized duplicated hard-coded values.
- Created servo calibration script for quicker calibration and servo support.- 
- A more feature rich motor and peripheral expansion board using a Teensy 4.0 supporting aux servos, foot sensors, NeoPixls, etc.
- 8x 18650 user accessable battery holder with onboard BMS (battery management system). 
  
### Credits
- Simulation, kinematics, bezier curves, and machine learning are largely based on the work from Spot Mini Mini: https://github.com/OpenQuadruped/spot_mini_mini 
- Spot Mini Mini is based on SpotMicroAI: https://spotmicroai.readthedocs.io/en/latest/
- Frame and leg linkage design inspired by Baris Alp's Kangel robot dog: https://grabcad.com/library/diy-quadruped-robot-1

### Communities
- Quadrupedalism: https://quadrupedalism.com/
- SpotMicroAI Discord: https://discord.gg/m5fsENhT

