import numpy as np

# Inverse kinematics.
# Calculation relies on cosine rule.

# Theta angles are absolute.
# Other greeks are relative.

#####################################
# User input: end effector coordinates (x, y).
foot_x = 34.14
foot_y = -163.78

#####################################
# Frame parameters, determined by the physical model.
# Lower servo origin, tibia crank arm.
Ax = -20
Ay = -20

# Upper servo origin, femur crank arm.
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

Hx = foot_x
Hy = foot_y
L1 = np.sqrt(np.square(Dx - Ax) + np.square(Dy - Ay))
DH = np.sqrt(np.square(Hx - Dx) + np.square(Hy - Dy))
theta1 = np.arcsin((Dy - Ay) / L1)

beta1 = np.arccos((np.square(L8) + np.square(DH) - np.square(L10)) / (2 * L8 * DH))
beta2 = np.arccos((np.square(L10) + np.square(L8) - np.square(DH)) / (2 * L10 * L8))

theta7 = np.arctan((Hy - Dy) / (Hx - Dx))
theta4 = theta7 + beta1
theta6 = beta2 + theta4

beta3 = np.pi - beta2
DF = np.sqrt(np.square(L8) + np.square(L9) - 2 * L8 * L9 * np.cos(beta3))

beta5 = np.arccos((np.square(DF) + np.square(L8) - np.square(L9)) / (2 * DF * L8))
beta6 = np.arccos((np.square(L6) + np.square(DF) - np.square(L7)) / (2 * L6 * DF))
theta5 = beta6 + beta5 + theta4

beta4 = np.arccos((np.square(L4) + np.square(L6) - np.square(L5)) / (2 * L4 * L6))
theta3 = beta4 + theta5

beta9 = np.pi - theta3 + theta1
AC = np.sqrt(np.square(L1) + np.square(L4) - 2 * L1 * L4 * np.cos(beta9))

beta7 = np.arccos((np.square(L2) + np.square(AC) - np.square(L3)) / (2 * L2 * AC))
beta8 = np.arccos((np.square(AC) + np.square(L1) - np.square(L4)) / (2 * AC * L1))

theta2 = theta1 + beta8 + beta7

Bx = Ax + L2 * np.cos(theta2)
By = Ay + L2 * np.sin(theta2)
Cx = Dx + L4 * np.cos(theta3)
Cy = Dy + L4 * np.sin(theta3)
Ex = Dx + L6 * np.cos(theta5)
Ey = Dy + L6 * np.sin(theta5)
Gx = Dx + L8 * np.cos(theta4)
Gy = Dy + L8 * np.sin(theta4)
Fx = Gx + L9 * np.cos(theta6)
Fy = Gy + L9 * np.sin(theta6)
Hx = Gx + L10 * np.cos(theta6 - np.pi)
Hy = Gy + L10 * np.sin(theta6 - np.pi)

print("A = (%.2f, %.2f)" % (Ax, Ay))
print("B = (%.2f, %.2f)" % (Bx, By))
print("C = (%.2f, %.2f)" % (Cx, Cy))
print("D = (%.2f, %.2f)" % (Dx, Dy))
print("E = (%.2f, %.2f)" % (Ex, Ey))
print("F = (%.2f, %.2f)" % (Fx, Fy))
print("G = (%.2f, %.2f)" % (Gx, Gy))
print("H = (%.2f, %.2f)" % (Hx, Hy))
print("")
print("DH = %.2f" % (DH))
print("DF = %.2f" % (DF))
print("AC = %.2f" % (AC))
print("")
print("beta1 = %.2f" % (np.degrees(beta1)))
print("beta2 = %.2f" % (np.degrees(beta2)))
print("beta3 = %.2f" % (np.degrees(beta3)))
print("beta4 = %.2f" % (np.degrees(beta4)))
print("beta5 = %.2f" % (np.degrees(beta5)))
print("beta6 = %.2f" % (np.degrees(beta6)))
print("beta7 = %.2f" % (np.degrees(beta7)))
print("beta8 = %.2f" % (np.degrees(beta8)))
print("beta9 = %.2f" % (np.degrees(beta9)))
print("")
print("theta9 = %.2f" % (np.degrees(theta9)))
print("theta8 = %.2f" % (np.degrees(theta8)))
print("theta6 = %.2f" % (np.degrees(theta6)))
print("theta5 = %.2f" % (np.degrees(theta5)))
print("theta4 = %.2f" % (np.degrees(theta4)))
print("theta2 = %.2f" % (np.degrees(theta2)))
print("theta1 = %.2f" % (np.degrees(theta1)))
print("")
print("tibia crank angle = %.2f" % (np.degrees(theta2)))
print("femur crank angle = %.2f" % (np.degrees(theta5)))