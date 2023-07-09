from joint_tracker import JointTracker
from animate_joint import JointAnimation
from animate_data import RecordedData

if __name__ == "__main__":
    # Ao vivo
    joint_tracker = JointTracker(
        "C3:B0:42:88:FE:07", "F6:8C:51:58:97:63", calculation_time=0.5, record=False
    )
    joint_animation = JointAnimation(
        joint_tracker, [1, 1], frames=5000, xyz_lim=[[-2.5, 2.5]] * 3
    )
    joint_animation.animate()

    # Gravação
    # recorded_data = RecordedData(
    #    "./graphics_video/end/microbit_data_0.xlsx",
    #    "./graphics_video/end/microbit_data_1.xlsx",
    #    calculation_time=0.5,
    #    record=False,
    # )
    # joint_animation = JointAnimation(
    #    recorded_data, [1, 1], frames=None, xyz_lim=[[-2.5, 2.5]] * 3
    # joint_animation.export_animation()
