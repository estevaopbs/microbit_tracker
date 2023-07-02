from joint_tracker import JointTracker
from animate_joint import JointAnimation

if __name__ == "__main__":
    joint_tracker = JointTracker(
        "C3:B0:42:88:FE:07", "F6:8C:51:58:97:63", calculation_time=0.5, record=False
    )
    joint_animation = JointAnimation(
        joint_tracker, [1, 1], frames=5000, xyz_lim=[[-3, 3]] * 3
    )
    joint_animation.animate()
