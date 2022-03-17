# Zuko - A quadruped robot dog!

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v2.2/cad/zuko-v2.2-cad-render-side.png" width="640"> | 
------------ | 
Frame v2.2 CAD design. |

<img src="https://github.com/reubenstr/Zuko/blob/main/images/general/zuko-mini-australian-shepard.jpg" width="640"> | 
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
Frame v2.1 first time standing. |

<img src="https://github.com/reubenstr/Zuko/blob/main/images/v2.1/assembling/zuko-v2.1-linked-leg-bridge-bearing-assembly-front.jpg" width="640"> | 
------------ | 
Frame v2.1 upper leg's servo horn, linked leg bridge, and bridge bearing press fit together. |


[![Watch the video](https://img.youtube.com/vi/D2snHtP22o0/hqdefault.jpg)](https://youtu.be/D2snHtP22o0) | 
------------ | 
Video: frame v2.1 first tests |

[![Watch the video](https://img.youtube.com/vi/J1g2Bx3SKjw/hqdefault.jpg)](https://youtu.be/J1g2Bx3SKjw) | 
------------ | 
Video: Sydney meets Zuko |

### Status

**Development.**  

**V2.2 Frame is being tested and is nearly ready for replication**

### General

Zuko uses the ROS2 framework for communication, launching, and 3rd party addons. Zuko uses a Rasberry Pi 4 with a custom expansion board for motor and peripheral control via a Playstation 4 controller.

### Current Major Tasks 

- Complete ROS2 tasks - in progress
- Fab/test expansion-board PCB
- Update BOM
- Create PCB BOM
- Upload STL's
- Create print time/weight sheet
- Create how to documentation for end user

### ROS2 Workspace 

ROS2 source code is contained the quad_ws directory.

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
- SpotMicroAI Discord: https://discord.gg/m5fsENhT
- Quadrupedalism: https://quadrupedalism.com/

