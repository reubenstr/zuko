"""
Matplotlib Animation Example

author: Jake Vanderplas
email: vanderplas@astro.washington.edu
website: http://jakevdp.github.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
#from forward_kinematics import *

# First set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
ax = plt.axes(xlim=(-50, 100), ylim=(-25, 50))
line, = ax.plot([], [], lw=2)

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
    L2 = 20
    L3 = 30
    L4 = 28
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

    return ([Ax, Bx, Cx, Dx, Ax], [Ay, By, Cy, Dy, Ax])

# initialization function: plot the background of each frame
def init():
    line.set_data([], [])
    return line,

# animation function.  This is called sequentially
def animate(i):
    p = forward_kinematics(90 + i, 0);
    points.set_data(p)


if __name__ == '__main__':
    points, = ax.plot([], [], '-o', ms=round(7 * 1), lw=2, color='b', mfc='red')

    # call the animator.  blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=200, interval=100, blit=False)

    # save the animation as an mp4.  This requires ffmpeg or mencoder to be
    # installed.  The extra_args ensure that the x264 codec is used, so that
    # the video can be embedded in html5.  You may need to adjust this for
    # your system: for more information, see
    # http://matplotlib.sourceforge.net/api/animation_api.html
    # anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

    plt.show()