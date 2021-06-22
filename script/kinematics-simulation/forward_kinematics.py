import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# Forward kinematics model and simulation.
# Angles in degrees.
# Calculation relies on cosine rule.



def forward_kinematics(angle_femur, angle_tibia):

    #####################################
    # User input, two servo angles.

    servo_angle_femur = 0
    servo_angle_tibia = angle_femur
    #####################################

    #####################################
    # Frame parameters, set my physical model.

    # Lower servo origin, controls tibia.
    Ax = -20
    Ay = -20

    # Upper servo origin, controls femur.
    Dx = 0
    Dy = 0

    # Link lengths
    L2 = 23
    L3 = 30
    L4 = 23
    #####################################


    #####################################
    # First four-bar linkage calculation.

    L1 = np.sqrt(np.square(Dx - Ax) + np.square(Dy - Ay))

    theta1 = np.arcsin((Dy - Ay) / L1)
    theta2 = np.radians(servo_angle_tibia)

    AC = np.sqrt(np.square(L1) + np.square(L2) - 2 * L1 * L2 * np.cos(theta1 - theta2))
    beta = np.arccos((np.square(L1) + np.square(AC) - np.square(L2)) / (2 * L1 * AC));
    psi = np.arccos((np.square(L3) + np.square(AC) - np.square(L4)) / (2 * L3 * AC));
    lamda = np.arccos((np.square(L4) + np.square(AC) - np.square(L3)) / (2 * L4 * AC));

    theta3 = psi - beta + theta1;
    theta4 = np.pi - lamda - beta + theta1;

    Bx = Ax + L2 * np.cos(theta2)
    By = Ay + L2 * np.sin(theta2)
    Cx = Dx + L4 * np.cos(theta4)
    Cy = Dy + L4 * np.sin(theta4)
    #####################################

    #####################################
    # Print results.

    print("B = (%.2f, %.2f)" % (Bx, By))
    print("C = (%.2f, %.2f)" % (Cx, Cy))

    print("AC:", np.around(AC, 2))
    print("beta:", np.around(np.degrees(beta), 2))
    print("psi:", np.around(np.degrees(psi), 2))
    print("lamda:", np.around(np.degrees(lamda), 2))

    print("theta1:", np.around(np.degrees(theta1), 2))
    print("theta2:", np.around(np.degrees(theta2), 2))
    print("theta3:", np.around(np.degrees(theta3), 2))
    print("theta4:", np.around(np.degrees(theta4), 2))
