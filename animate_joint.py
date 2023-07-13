import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from joint_tracker import JointTracker


class JointAnimation:
    def __init__(self, joint_tracker: JointTracker, lenghts, frames, xyz_lim):
        self.joint_tracker = joint_tracker
        self.lengths = lenghts
        self.frames = frames
        self.frame = 0
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.xyz_lim = xyz_lim

    def update(self, frame):
        self.frame = frame
        self.joint_tracker.update()
        self.ax.clear()
        self.ax.set_xlim(self.xyz_lim[0])
        self.ax.set_ylim(self.xyz_lim[1])
        self.ax.set_zlim(self.xyz_lim[2])
        last_point = np.array([0, 0, 0])
        for vector, length in zip(self.joint_tracker.vectors, self.lengths):
            self.ax.quiver(*last_point, *vector * length)
            last_point = last_point + vector * length
        return

    def animate(self):
        self.animation = animation.FuncAnimation(
            self.fig, self.update, repeat=False, frames=self.frames
        )
        plt.show()
#