import numpy as np

# Forward kinematics.
# Calculation relies on cosine rule.

# Four-bar linkage solution process inspired by: https://www.youtube.com/watch?v=4O-XPJ7flLU&t=321s
# Animation source code inspiration: DIYwalkers.com


#####################################
# User input, two servo angles.

servo_angle_femur = -45 # degrees
servo_angle_tibia = 150  # degress

#####################################
# Frame parameters, set my physical model.
# Lower servo origin, tibia control.
Ax = -20
Ay = -20

# Upper servo origin, femur control.
Dx = 0
Dy = 0

# Link lengths
L2 = 23
L3 = 30
L4 = 23
L5 = 22
L6 = 23
L7 = 100
L8 = 100
L9 = 23
L10 = 100


#####################################
# Calculations
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

kappa = np.arccos((np.square(L4) + np.square(L6) - np.square(L5)) / (2 * L4 * L6))

theta5 = np.radians(servo_angle_femur)
theta6 = theta4 - kappa

EG = np.sqrt(np.square(L6) + np.square(L8) - 2 * L6 * L8 * np.cos(theta6 - theta5))
mu = np.arccos((np.square(L8) + np.square(EG) - np.square(L6)) / (2 * L8 * EG));
omega = np.arccos((np.square(L7) + np.square(EG) - np.square(L9)) / (2 * L7 * EG));
rho = np.arccos((np.square(EG) + np.square(L9) - np.square(L7)) / (2 * EG * L9));

theta7 = mu - omega + theta5;
theta8 = np.pi - rho - mu + theta5;

Ex = Dx + L6 * np.cos(theta6)
Ey = Dy + L6 * np.sin(theta6)
Gx = Dx + L8 * np.cos(theta5)
Gy = Dy + L8 * np.sin(theta5)
Fx = Gx + L9 * np.cos(theta8)
Fy = Gy + L9 * np.sin(theta8)
Hx = Gx + L10 * np.cos(theta8 - np.pi)
Hy = Gy + L10 * np.sin(theta8 - np.pi)


#####################################
# Print results.

print("A = (%.2f, %.2f)" % (Ax, Ay))
print("B = (%.2f, %.2f)" % (Bx, By))
print("C = (%.2f, %.2f)" % (Cx, Cy))
print("D = (%.2f, %.2f)" % (Dx, Dy))
print("E = (%.2f, %.2f)" % (Ex, Ey))
print("F = (%.2f, %.2f)" % (Fx, Fy))
print("G = (%.2f, %.2f)" % (Gx, Gy))
print("H = (%.2f, %.2f)" % (Hx, Hy))

print("AC:", np.around(AC, 2))
print("EG:", np.around(EG, 2))

print("beta = %.2f" % (np.degrees(beta)))
print("psi = %.2f" % (np.degrees(psi)))
print("lamda = %.2f" % (np.degrees(lamda)))
print("kappa = %.2f" % (np.degrees(kappa)))
print("mu = %.2f" % (np.degrees(mu)))
print("omega = %.2f" % (np.degrees(omega)))
print("rho = %.2f" % (np.degrees(rho)))

print("theta1 = %.2f" % (np.degrees(theta1)))
print("theta2 = %.2f" % (np.degrees(theta2)))
print("theta3 = %.2f" % (np.degrees(theta3)))
print("theta4 = %.2f" % (np.degrees(theta4)))
print("theta5 = %.2f" % (np.degrees(theta5)))
print("theta6 = %.2f" % (np.degrees(theta6)))
print("theta7 = %.2f" % (np.degrees(theta7)))
print("theta8 = %.2f" % (np.degrees(theta8)))