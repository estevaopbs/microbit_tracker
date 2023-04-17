import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from joint_tracker import JointTracker


class JointAnimation:
    def __init__(self, joint_tracker: JointTracker, lenghts, frames):
        self.joint_tracker = joint_tracker
        self.lengths = lenghts
        self.frames = frames
        self.frame = 0
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        # self.joint_angle_text = self.ax.text2D(
        #    0.85, 0, '', transform=self.ax.transAxes, fontsize=14,
        #    verticalalignment='top')

    def update(self, frame):
        self.frame = frame
        self.joint_tracker.update()
        self.ax.clear()
        self.ax.set_xlim([-1.5, 1.5])
        self.ax.set_ylim([-1.5, 1.5])
        self.ax.set_zlim([-1.5, 1.5])
        self.ax.quiver(*np.array([0, 1, 0]),
                       *self.joint_tracker.arm_vector * self.lengths[0])
        self.ax.quiver(0, 0, 0,
                       *self.joint_tracker.forearm_vector * self.lengths[1])
        # self.joint_angle_text.set_text(
        #    f'{np.angle(self.joint_tracker.joint_angle)}')
        return

    def animate(self):
        self.animation = animation.FuncAnimation(self.fig, self.update, repeat=False,
                                                 frames=self.frames)
        plt.show()
