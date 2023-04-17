import numpy as np
from kaspersmicrobit import KaspersMicrobit
from kaspersmicrobit.services.leddisplay import Image
from time import sleep


def get_angle(v1, v2):
    dot_product = np.dot(v1, v2)
    v1_mag = np.linalg.norm(v1)
    v2_mag = np.linalg.norm(v2)
    cos_angle = dot_product / (v1_mag * v2_mag)
    angle = np.arccos(cos_angle)
    cross_product = np.cross(v1, v2)
    # print(v1, v2, cross_product)
    if int(cross_product) < 0:
        angle = 2 * np.pi - angle
    return angle


class JointTracker:
    def __init__(self, arm_mb_id, forearm_mb_id) -> None:
        self.arm_mb, self.forearm_mb = self.get_connection(
            arm_mb_id, forearm_mb_id)
        self.arm_vector = None
        self.forearm_vector = None
        self.gravity_north_angle = None
        self.joint_angle = None
        # self.startup()

    def startup(self):
        for image in (Image.CLOCK12, Image.CLOCK1, Image.CLOCK2, Image.CLOCK3,
                      Image.CLOCK4, Image.CLOCK5, Image.CLOCK6, Image.CLOCK7,
                      Image.CLOCK8, Image.CLOCK9, Image.CLOCK10, Image.CLOCK11,
                      Image.CLOCK12):
            self.arm_mb.led.show(image)
            sleep(1)
        self.gravity_north_angle = get_angle(
            self._get_magnetometer(self.arm_mb),
            self._get_accelerometer(self.arm_mb)
        )

    @staticmethod
    def _get_accelerometer(mb: KaspersMicrobit):
        data = mb.accelerometer.read()
        return np.array([data.x, data.y, data.z])

    @staticmethod
    def _get_magnetometer(mb: KaspersMicrobit):
        data = mb.magnetometer.read_data()
        return np.array([data.x, data.y, data.z])

    @staticmethod
    def get_connection(arm_mb_id, forearm_mb_id):
        mbs = []
        for mb in (KaspersMicrobit(arm_mb_id), KaspersMicrobit(forearm_mb_id)):
            mb.connect()
            mbs.append(mb)
        return mbs[0], mbs[1]

    def update(self):
        arm_north = self._get_magnetometer(self.arm_mb)
        forearm_north = self._get_magnetometer(self.forearm_mb)

        arm_north_yz = np.array([arm_north[1], arm_north[2]])

        forearm_north_yz = np.array([forearm_north[1], forearm_north[2]])
        # forearm_north_yz = forearm_north_yz / norm(forearm_north_yz)

        # Calculate the angle between the north vector and the forearm
        theta = get_angle(forearm_north_yz, np.array([-1, 0]))

        r = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])
        self.forearm_vector = np.dot(
            r, np.array([0, arm_north[1], arm_north[2]]) / np.linalg.norm(arm_north_yz))
        self.arm_vector = np.array([0, -1, 0])
        # self.joint_angle = get_angle(self.arm_vector, self.forearm_vector)
        # phi = np.arccos(np.dot(np.array([0, 1, 0]), vector)) * 180 / np.pi
