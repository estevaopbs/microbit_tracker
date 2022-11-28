import numpy as np
from numpy.linalg import norm


def get_angles(arm_north, forearm_north):
    """
    Calculates the angles of the arm and forearm.
    :param arm_north: The north vector of the arm.
    :param forearm_north: The north vector of the forearm.
    :return: The angles of the arm and forearm.
    """

    # Calculate the angle of the arm
    arm_north = arm_north / norm(arm_north)
    arm_theta = np.arccos(np.dot(arm_north, [-1, 0, 0])) * 180 / np.pi
    arm_phi = np.arccos(np.dot(arm_north, [0, 1, 0])) * 180 / np.pi

    # Calculate the angle of the forearm
    forearm_north = forearm_north / norm(forearm_north)
    forearm_theta = np.arccos(np.dot(forearm_north, [-1, 0, 0])) * 180 / np.pi
    forearm_phi = np.arccos(np.dot(forearm_north, [0, 1, 0])) * 180 / np.pi

    return arm_theta, arm_phi, forearm_theta, forearm_phi
