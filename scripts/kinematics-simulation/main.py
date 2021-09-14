# Project Zuko
#
# Leg linkage forward Kinematic model simulation.
#
# Limitations: no mechanical collision detection.
#
# Note: switch from FK to IK no realized in code (TODO).

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

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


# Determine end effector (foot) position from the two input angles.
# angle_femur: femur servo position in degrees where the x-axis is zero degrees.
# angle_tibia: tibia servo position in degrees where the x-axis is zero degrees.
def forward_kinematics(angle_femur, angle_tibia):
    #####################################
    # Calculations
    L1 = np.sqrt(np.square(Dx - Ax) + np.square(Dy - Ay))

    theta1 = np.arcsin((Dy - Ay) / L1)
    theta2 = np.radians(angle_tibia)

    AC = np.sqrt(np.square(L1) + np.square(L2) - 2 * L1 * L2 * np.cos(theta1 - theta2))
    beta = np.arccos((np.square(L1) + np.square(AC) - np.square(L2)) / (2 * L1 * AC))
    psi = np.arccos((np.square(L3) + np.square(AC) - np.square(L4)) / (2 * L3 * AC))
    lamda = np.arccos((np.square(L4) + np.square(AC) - np.square(L3)) / (2 * L4 * AC))

    theta3 = psi - beta + theta1
    theta4 = np.pi - lamda - beta + theta1

    Bx = Ax + L2 * np.cos(theta2)
    By = Ay + L2 * np.sin(theta2)
    Cx = Dx + L4 * np.cos(theta4)
    Cy = Dy + L4 * np.sin(theta4)

    kappa = np.arccos((np.square(L4) + np.square(L6) - np.square(L5)) / (2 * L4 * L6))

    theta5 = np.radians(angle_femur)
    theta6 = theta4 - kappa

    EG = np.sqrt(np.square(L6) + np.square(L8) - 2 * L6 * L8 * np.cos(theta6 - theta5))
    mu = np.arccos((np.square(L8) + np.square(EG) - np.square(L6)) / (2 * L8 * EG))
    omega = np.arccos((np.square(L7) + np.square(EG) - np.square(L9)) / (2 * L7 * EG))
    rho = np.arccos((np.square(EG) + np.square(L9) - np.square(L7)) / (2 * EG * L9))

    theta7 = mu - omega + theta5
    theta8 = np.pi - rho - mu + theta5

    Ex = Dx + L6 * np.cos(theta6)
    Ey = Dy + L6 * np.sin(theta6)
    Gx = Dx + L8 * np.cos(theta5)
    Gy = Dy + L8 * np.sin(theta5)
    Fx = Gx + L9 * np.cos(theta8)
    Fy = Gy + L9 * np.sin(theta8)
    Hx = Gx + L10 * np.cos(theta8 - np.pi)
    Hy = Gy + L10 * np.sin(theta8 - np.pi)

    return [[[Ax, Bx, Cx, Dx, Ax], [Ay, By, Cy, Dy, Ax]],
            [[Cx, Ex], [Cy, Ey]],
            [[Hx, Gx, Fx, Ex, Dx, Gx], [Hy, Gy, Fy, Ey, Dy, Gy]],
            [[Hx], [Hy]]]


def inverse_kinematics(foot_x, foot_y):
    #####################################
    # Calculations
    Hx = foot_x
    Hy = foot_y
    L1 = np.sqrt(np.square(Dx - Ax) + np.square(Dy - Ay))
    DH = np.sqrt(np.square(Hx - Dx) + np.square(Hy - Dy))
    theta1 = np.arcsin((Dy - Ay) / L1)
    epsilon = np.arccos((np.square(L8) + np.square(DH) - np.square(L10)) / (2 * L8 * DH))
    nu = np.arccos((np.square(L10) + np.square(L8) - np.square(DH)) / (2 * L10 * L8))

    # Prevent divide by zero and process quadrant shift.
    if (Hx - Dx) != 0:
        theta9 = np.arctan((Hy - Dy) / (Hx - Dx))
        if theta9 > 0:
            theta9 = -(np.pi - theta9)
    else:
        theta9 = -np.pi / 2

    theta5 = theta9 + epsilon
    theta8 = nu + theta5

    delta = np.pi - nu
    DF = np.sqrt(np.square(L8) + np.square(L9) - 2 * L8 * L9 * np.cos(delta))

    beta1 = np.arccos((np.square(DF) + np.square(L8) - np.square(L9)) / (2 * DF * L8))
    beta2 = np.arccos((np.square(L6) + np.square(DF) - np.square(L7)) / (2 * L6 * DF))
    theta6 = beta2 + beta1 + theta5

    kappa = np.arccos((np.square(L4) + np.square(L6) - np.square(L5)) / (2 * L4 * L6))
    theta4 = kappa + theta6

    beta5 = np.pi - theta4 + theta1
    AC = np.sqrt(np.square(L1) + np.square(L4) - 2 * L1 * L4 * np.cos(beta5))

    beta3 = np.arccos((np.square(L2) + np.square(AC) - np.square(L3)) / (2 * L2 * AC))
    beta4 = np.arccos((np.square(AC) + np.square(L1) - np.square(L4)) / (2 * AC * L1))

    theta2 = theta1 + beta4 + beta3

    Bx = Ax + L2 * np.cos(theta2)
    By = Ay + L2 * np.sin(theta2)
    Cx = Dx + L4 * np.cos(theta4)
    Cy = Dy + L4 * np.sin(theta4)
    Ex = Dx + L6 * np.cos(theta6)
    Ey = Dy + L6 * np.sin(theta6)
    Gx = Dx + L8 * np.cos(theta5)
    Gy = Dy + L8 * np.sin(theta5)
    Fx = Gx + L9 * np.cos(theta8)
    Fy = Gy + L9 * np.sin(theta8)
    Hx = Gx + L10 * np.cos(theta8 - np.pi)
    Hy = Gy + L10 * np.sin(theta8 - np.pi)

    return [[[Ax, Bx, Cx, Dx, Ax], [Ay, By, Cy, Dy, Ax]],
            [[Cx, Ex], [Cy, Ey]],
            [[Hx, Gx, Fx, Ex, Dx, Gx], [Hy, Gy, Fy, Ey, Dy, Gy]],
            [[Hx], [Hy]]]


# initialization function: plot the background of each frame
def init():
    pass


# Animation function called every tick.
def animate(i):
    global footpath_x
    global footpath_y
    global femur_servo_angle
    global tibia_servo_angle
    global tibia_servo_angle_start
    global tibia_servo_angle_max
    global femur_servo_angle_max
    global direction

    if direction:
        tibia_servo_angle = tibia_servo_angle + 1
    else:
        tibia_servo_angle = tibia_servo_angle - 1

    if tibia_servo_angle == tibia_servo_angle_start:
        direction = not direction
        femur_servo_angle = femur_servo_angle - 1

    if tibia_servo_angle == tibia_servo_angle_max:
        direction = not direction
        femur_servo_angle = femur_servo_angle - 1

    if femur_servo_angle == femur_servo_angle_max:
        anim.event_source.stop()

    # Choose FK or IK
    result_points = forward_kinematics(femur_servo_angle, tibia_servo_angle);
    #result_points = inverse_kinematics(50 - i, -50 - i);

    line1.set_data(result_points[0])
    line2.set_data(result_points[1])
    line3.set_data(result_points[2])

    footpath_x = np.append(footpath_x, result_points[3][0])
    footpath_y = np.append(footpath_y, result_points[3][1])
    foot_path.set_data(footpath_x, footpath_y)


if __name__ == '__main__':
    # Animation variables.
    tibia_servo_angle_start = 75
    tibia_servo_angle_max = 145
    tibia_servo_angle = tibia_servo_angle_start

    femur_servo_angle_start = -10
    femur_servo_angle_max = -75
    femur_servo_angle = femur_servo_angle_start

    footpath_x = np.array([1])
    footpath_y = np.array([1])

    direction = True

    fig = plt.figure()
    ax = plt.axes(xlim=(-100, 150), ylim=(-200, 50))
    plt.title("Left Leg Forward Kinematics Simulation")
    plt.xlabel('X')
    plt.ylabel('Y')
    line1, = ax.plot([], [], '-o', ms=round(7), lw=2, color='b', mfc='red')
    line2, = ax.plot([], [], '-o', ms=round(7), lw=2, color='b', mfc='red')
    line3, = ax.plot([], [], '-o', ms=round(7), lw=2, color='b', mfc='red')
    foot_path, = ax.plot([], [], '-o', lw=0, ms=2, alpha=.5, mfc='red', mec='red')

    # Call the animator.  blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, animate, init_func=init, frames=400, interval=0, blit=False)

    # save the animation as an mp4.  This requires ffmpeg or mencoder to be
    # installed.  The extra_args ensure that the x264 codec is used, so that
    # the video can be embedded in html5.  You may need to adjust this for
    # your system: for more information, see
    # http://matplotlib.sourceforge.net/api/animation_api.html
    # anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

    plt.show()
