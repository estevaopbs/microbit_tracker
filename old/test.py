import matplotlib.pyplot as plt
import matplotlib.animation as animation
from kaspersmicrobit import KaspersMicrobit
import numpy as np
from numpy.linalg import norm
from os import system


def get_angle(arm_north, forearm_north):
    # Calculate the angles of the arm
    arm_north_yz = arm_north * np.array([0, 1, 1])
    arm_north_yz = arm_north_yz / norm(arm_north_yz)

    # Calculate the angles of the forearm
    forearm_north_yz = forearm_north * np.array([0, 1, 1])
    forearm_north_yz = forearm_north_yz / norm(forearm_north_yz)

    theta = np.arccos(np.dot(forearm_north_yz, [0, -1, 0]))
    r = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)],
                 [0, np.sin(theta), np.cos(theta)]])
    vector = np.dot(r, arm_north_yz)
    phi = np.arccos(np.dot(np.array([0, 1, 0]), vector)) * 180 / np.pi
    return phi


if __name__ == '__main__':
    # Microbit 0 - Arm
    ADDRESS_0 = 'C3:B0:42:88:FE:07'
    arm_mb = KaspersMicrobit(ADDRESS_0)
    arm_mb.connect()
    # Microbit 1 - Forearm
    ADDRESS_1 = 'F6:8C:51:58:97:63'
    forearm_mb = KaspersMicrobit(ADDRESS_1)
    forearm_mb.connect()
    while True:
        arm_mag = arm_mb.magnetometer.read_data()
        arm_mvec = np.array([arm_mag.x, arm_mag.y, arm_mag.z])
        forearm_mag = forearm_mb.magnetometer.read_data()
        forearm_mvec = np.array([forearm_mag.x, forearm_mag.y, forearm_mag.z])
        angle = get_angle(arm_mvec, forearm_mvec)
        system('cls')
        print(arm_mvec)
        print(forearm_mvec)
        print(angle)
