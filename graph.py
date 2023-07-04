import pandas as pd
import numpy as np
from joint_tracker import get_angle
import time
from matplotlib import pyplot as plt

TIMESPAN = 0.5


class ArticulationState:
    get_speeds = lambda x, y: [0, 0]
    get_accs = lambda x, y: [0, 0]

    def __init__(self, vectors, angles, upd_time, ref_state) -> None:
        self.vectors = vectors
        self.angles = angles
        self.upd_time = upd_time
        self.speeds = ArticulationState.get_speeds(self, ref_state)
        self.accs = ArticulationState.get_accs(self, ref_state)

    @staticmethod
    def _get_speeds(state0, state1):
        speeds = []
        for a0, a1 in zip(state0.angles, state1.angles):
            speeds.append((a0 - a1) / (state0.upd_time - state1.upd_time))
        return speeds

    @staticmethod
    def _get_accs(state0, state1):
        accs = []
        for a0, a1 in zip(state0.speeds, state1.speeds):
            accs.append((a0 - a1) / (state0.upd_time - state1.upd_time))
        return accs


def _get_first_state(vectors, angles, upd_time, last_states):
    state = ArticulationState(vectors, angles, upd_time, None)
    ArticulationState.get_speeds = ArticulationState._get_speeds
    ArticulationState.get_accs = ArticulationState._get_accs
    global get_state_fn
    get_state_fn = _get_state
    return state


def _get_state(vectors, angles, upd_time, last_states):
    return ArticulationState(vectors, angles, upd_time, last_states[0])


def get_magnetometer(microbit_row):
    return np.array(
        [
            microbit_row["MAGNETOMETER_X"],
            microbit_row["MAGNETOMETER_Y"],
            microbit_row["MAGNETOMETER_Z"],
        ]
    )


def get_accelerometer(microbit_row):
    return np.array(
        [
            microbit_row["ACCELEROMETER_X"],
            microbit_row["ACCELEROMETER_Y"],
            microbit_row["ACCELEROMETER_Z"],
        ]
    )


def get_upd_time(microbit_row):
    return microbit_row["TIME"]


def get_state_main(*microbit_rows, last_states):
    vectors = [np.array([0, -1, 0])]
    angles = [np.pi / 2]
    upd_time = get_upd_time(microbit_rows[0])
    north0 = get_magnetometer(microbit_rows[0])
    north0_yz = np.array([north0[1], north0[2]])
    for microbit_row in microbit_rows[1:]:
        northn = get_magnetometer(microbit_row)
        northn_yz = np.array([northn[1], northn[2]])
        theta = get_angle(np.array([-1, 0]), northn_yz)
        r = np.array(
            [
                [1, 0, 0],
                [0, np.cos(theta), np.sin(theta)],
                [0, -np.sin(theta), np.cos(theta)],
            ]
        )
        narm_vector = np.dot(r, np.array([0, north0_yz[0], north0_yz[1]]))
        vectors.append(narm_vector)
        angles.append(theta)
    return get_state_fn(vectors, angles, upd_time, last_states)


def get_state_fullgravity2d(*microbit_rows, last_states):
    def get_positions(acc, fixed_gravity):
        acc_yz = np.array([acc[1], acc[2]])
        theta = get_angle(acc_yz, np.array([1, 0]))
        rtheta = np.array(
            [
                [1, 0, 0],
                [0, np.cos(theta), -np.sin(theta)],
                [0, np.sin(theta), np.cos(theta)],
            ]
        )
        return np.dot(rtheta, fixed_gravity), theta

    fixed_gravity = np.array([0, 0, -1])
    vectors = []
    angles = []
    upd_time = get_upd_time(microbit_rows[0])
    for microbit_row in microbit_rows:
        acc = get_accelerometer(microbit_row)
        vector, angle = get_positions(acc, fixed_gravity)
        vectors.append(vector)
        angles.append(angle)
    angles = [angles[0]] + [angle - angles[n] for n, angle in enumerate(angles[1:])]
    return get_state_fn(vectors, angles, upd_time, last_states)


# Acelerômetro
# microbit_data_0 = pd.read_excel("./graphics_video/microbit_acc_0.xlsx")
# microbit_data_1 = pd.read_excel("./graphics_video/microbit_acc_1.xlsx")

# Magnetômetro
microbit_data_0 = pd.read_excel("./graphics_video/microbit_mag_0.xlsx")
microbit_data_1 = pd.read_excel("./graphics_video/microbit_mag_1.xlsx")


get_state_fn = _get_first_state
all_states = []
last_states = []
for mb0, mb1 in zip(microbit_data_0.iterrows(), microbit_data_1.iterrows()):
    i0, data0 = mb0
    i1, data1 = mb1

    # Acelerômetro
    # state = get_state_fullgravity2d(data0, data1, last_states=last_states)

    # Magnetômetro
    state = get_state_main(data0, data1, last_states=last_states)

    for n, _state in enumerate(last_states):
        if state.upd_time - _state.upd_time <= TIMESPAN:
            last_states = last_states[n:]
            break
    last_states.append(state)
    all_states.append(state)

fig = plt.figure()
ax_angles = fig.add_subplot(311)
ax_speeds = fig.add_subplot(312)
ax_accs = fig.add_subplot(313)
times = [state.upd_time - all_states[0].upd_time for state in all_states]

# Angles
angles_0 = [np.degrees(state.angles[0]) for state in all_states]
angles_1 = [np.degrees(state.angles[1]) for state in all_states]
# ax_angles.plot(times, angles_0, label="Articulação 0")
ax_angles.plot(times, angles_1, label="Articulação 1")
ax_angles.set_title("Ângulos")
ax_angles.set_xlabel("Tempo (s)")
ax_angles.set_ylabel("Ângulo (graus)")
ax_angles.legend()
ax_angles.legend(loc="upper right")


# Speeds
speeds_0 = [state.speeds[0] for state in all_states]
speeds_1 = [state.speeds[1] for state in all_states]
# ax_speeds.plot(times, speeds_0, label="Articulação 0")
ax_speeds.plot(times, speeds_1, label="Articulação 1")
ax_speeds.set_title("Velocidades Angulares")
ax_speeds.set_xlabel("Tempo (s)")
ax_speeds.set_ylabel("Velocidade Angular (rad/s)")
ax_speeds.legend()
ax_speeds.legend(loc="upper right")


# Accs
accs_0 = [state.accs[0] for state in all_states]
accs_1 = [state.accs[1] for state in all_states]
# ax_accs.plot(times, accs_0, label="Articulação 0")
ax_accs.plot(times, accs_1, label="Articulação 1")
ax_accs.set_title("Acelerações Angulares")
ax_accs.set_xlabel("Tempo (s)")
ax_accs.set_ylabel("Aceleração Angular (rad/s²)")
ax_accs.legend()
ax_accs.legend(loc="upper right")


plt.subplots_adjust(hspace=0.5)
manager = plt.get_current_fig_manager()
# manager.full_screen_toggle()
plt.show()
