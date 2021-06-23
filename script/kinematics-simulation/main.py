import numpy
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

# from forward_kinematics import *

# declare variables



def forward_kinematics(angle_femur, angle_tibia):
    #####################################
    # User input, two servo angles.

    servo_angle_femur = angle_femur
    servo_angle_tibia = angle_tibia
    #####################################

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

    theta5 = np.radians(servo_angle_femur)
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

    return np.array([[[Ax, Bx, Cx, Dx, Ax], [Ay, By, Cy, Dy, Ax]],
                     [[Cx, Ex], [Cy, Ey]],
                     [[Hx, Gx, Fx, Ex, Dx, Gx], [Hy, Gy, Fy, Ey, Dy, Gy]],
                     [[Hx], [Hy]]])


# initialization function: plot the background of each frame
def init():
    pass


# Animation function called every tick.
def animate(i):
    global footpath_x
    global footpath_y
    global angle_femur
    global angle_tibia
    global angle_tibia_start
    global angle_tibia_max
    global angle_femur_max

    angle_tibia = angle_tibia + 1

    if angle_tibia == angle_tibia_max:
        angle_tibia = angle_tibia_start
        angle_femur = angle_femur - 1

    if angle_femur == angle_femur_max:
        anim.event_source.stop()

    result_points = forward_kinematics(angle_femur, angle_tibia);

    line1.set_data(result_points[0])
    line2.set_data(result_points[1])
    line3.set_data(result_points[2])

    footpath_x = np.append(footpath_x, result_points[3][0])
    footpath_y = np.append(footpath_y, result_points[3][1])
    foot_path.set_data(footpath_x, footpath_y)

if __name__ == '__main__':

    angle_tibia_start = 75
    angle_tibia_max = 135
    angle_tibia = angle_tibia_start

    angle_femur_start = -10
    angle_femur_max = -75
    angle_femur = angle_femur_start

    footpath_x = np.array([1])
    footpath_y = np.array([1])
    fig = plt.figure()
    ax = plt.axes(xlim=(-100, 150), ylim=(-200, 50))
    line1, = ax.plot([], [], '-o', ms=round(7), lw=2, color='b', mfc='red')
    line2, = ax.plot([], [], '-o', ms=round(7), lw=2, color='b', mfc='red')
    line3, = ax.plot([], [], '-o', ms=round(7), lw=2, color='b', mfc='red')
    foot_path, = ax.plot([], [], '-o', lw=0, ms=2, alpha=.5, mfc='red', mec='red')

    # call the animator.  blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=400, interval=0, blit=False)

    # save the animation as an mp4.  This requires ffmpeg or mencoder to be
    # installed.  The extra_args ensure that the x264 codec is used, so that
    # the video can be embedded in html5.  You may need to adjust this for
    # your system: for more information, see
    # http://matplotlib.sourceforge.net/api/animation_api.html
    # anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

    plt.show()
