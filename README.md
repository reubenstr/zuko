# Zuko
 A quadruped robot dog!

<img src="https://github.com/reubenstr/Zuko/blob/main/images/zuko-render-side.png" width="640">

Status:

**Development**


Tasks:

[] Update source

[] Connect motion controller to servos

[] Migrate to rPi

[x] Design frame

[x] Print frame

[x] Design PCBs

[x] Assembly PCBs


My contributions:
- Migration of source code from ROS1 to ROS2 (Robot Operating System) frameworks.
- Inverse kinematics for a linked leg system for OpenQuadruped / Spot Mini Mini based quadrupeds.
- A frame redesign using larger servos, providing more space for electronics, rework of the leg/hip system, and addition of aesthetic features (tail, etc).
- A more feature rich motor and peripheral expansion board using a Teensy 4.0 supporting aux servos, foot sensors, NeoPixls, etc.
- 8x 18650 user accessable battery holder with onboard BMS (battery management system). 
 
Credits:
- Simulation, kinematics, bezier curves, and machine learning are largely based on the work from Spot Mini Mini: https://github.com/OpenQuadruped/spot_mini_mini 
- Spot Mini Mini is based on SpotMicroAI: https://spotmicroai.readthedocs.io/en/latest/
 
