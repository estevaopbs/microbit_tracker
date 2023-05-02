from os import system
from time import sleep
from typing import List

import numpy as np
from kaspersmicrobit import KaspersMicrobit
from kaspersmicrobit.services.leddisplay import Image


def get_angle(v1, v2):
    v1_mag = np.linalg.norm(v1)
    v2_mag = np.linalg.norm(v2)
    v1_unit = v1 / v1_mag
    v2_unit = v2 / v2_mag
    cos_angle = np.dot(v1_unit, v2_unit)
    angle = np.arccos(cos_angle)
    cross_product = np.cross(v1_unit, v2_unit)
    if float(cross_product) < 0:
        angle = 2 * np.pi - angle
    return angle


class JointTracker:
    def __init__(self, *microbits: str | KaspersMicrobit) -> None:
        self.microbits = self.get_connection(microbits)
        self.vectors: List[np.ndarray(shape=3, dtype=float)] = []
        self.gravity_north_angle = None

        # self.startup()

    def startup(self):
        for image in (Image.CLOCK1, Image.CLOCK2, Image.CLOCK3,
                      Image.CLOCK4, Image.CLOCK5, Image.CLOCK6, Image.CLOCK7,
                      Image.CLOCK8, Image.CLOCK9, Image.CLOCK10, Image.CLOCK11,
                      Image.CLOCK12):
            self.arm_mb.led.show(image)
            sleep(1)
        self.gravity_north_angle = get_angle(
            self._get_magnetometer(self.microbits[0]),
            self._get_accelerometer(self.microbits[0])
        )

    @staticmethod
    def _get_accelerometer(mb: KaspersMicrobit) -> np.ndarray(shape=3, dtype=np.int16):
        data = mb.accelerometer.read()
        return np.array([data.x, data.y, data.z], dtype=np.int16)

    @staticmethod
    def _get_magnetometer(mb: KaspersMicrobit) -> np.ndarray(shape=3, dtype=np.int16):
        data = mb.magnetometer.read_data()
        return np.array([data.x, data.y, data.z], dtype=np.int16)

    @staticmethod
    def get_connection(microbits: List[KaspersMicrobit | str]) -> List[KaspersMicrobit]:
        connected_kms: List[KaspersMicrobit] = []
        for microbit in microbits:
            match microbit:
                case KaspersMicrobit():
                    microbit.connect()
                    connected_kms.append(microbit)
                case str():
                    _microbit = KaspersMicrobit(microbit)
                    _microbit.connect()
                    connected_kms.append(_microbit)
                case _:
                    raise TypeError(
                        f'Invalid type of {microbit}, it should be either KaspersMicrobit or str.')
        return connected_kms

    def update(self):
        system('cls')
        vectors = [np.array([0, -1, 0])]
        north0 = self._get_magnetometer(self.microbits[0])
        north0_yz = np.array([north0[1], north0[2]])
        for microbit in self.microbits[1:]:
            northn = self._get_magnetometer(microbit)
            northn_yz = np.array([northn[1], northn[2]])
            theta = get_angle(np.array([-1, 0]), northn_yz)
            r = np.array([[1, 0, 0], [0, np.cos(theta), np.sin(theta)],
                         [0, -np.sin(theta), np.cos(theta)]])  # testar matriz de rotação no sentido oposto
            narm_vector = np.dot(
                r, np.array([0, north0[1], north0[2]]))\
                / np.linalg.norm(north0_yz)
            vectors.append(narm_vector)
        self.vectors = vectors
        # rot_vectors = []
        # north0_xy = np.array([north0[0], north0[1]])
        # phi = get_angle(north0_xy, np.array([0, 1]))
        # rphi = np.array([[np.cos(phi), np.sin(phi), 0],
        #                 [-np.sin(phi), np.cos(phi), 0],
        #                 [0, 0, 1]])
        # north0_xz = np.array([north0[0], north0[2]])
        # gama = get_angle(north0_xz, np.array([0, 1]))
        # rgama = np.array([[np.cos(gama), 0, -np.sin(gama)],
        #                  [0, 1, 0],
        #                  [np.sin(gama), 0, np.cos(gama)]])
        # for vector in vectors:
        #    rot_vectors.append(np.dot(rgama, np.dot(rphi, vector)))
        # self.vectors = rot_vectors
