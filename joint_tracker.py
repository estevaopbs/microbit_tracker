from os import system
from time import sleep
from typing import List

import numpy as np
from kaspersmicrobit import KaspersMicrobit
from kaspersmicrobit.services.leddisplay import Image


# Recebe dois vetores bidimensionais e calcula o ângulo entre eles no
# sentido antihorário
def get_angle(v1, v2):
    # Normaliza os vetores
    v1_unit = v1 / np.linalg.norm(v1)
    v2_unit = v2 / np.linalg.norm(v2)

    # Toma o cosseno do ângulo a partir do produto escalar
    cos_angle = np.dot(v1_unit, v2_unit)

    # Encontra o ângulo a partir da função arcosseno
    # no intervalo 0 <= ângulo <= 180
    angle = np.arccos(cos_angle)

    # Encontra o seno do ângulo através do produto vetorial. Se o seno for
    # negativo (180 < ângulo < 360), então o ângulo é definido como
    # 360 - (ângulo encontrado por arcosseno).
    cross_product = np.cross(v1_unit, v2_unit)
    if float(cross_product) < 0:
        angle = 2 * np.pi - angle
    return angle


class JointTracker:
    fixed_north = np.array([0, 1, 0])

    def __init__(self, *microbits: str | KaspersMicrobit):
        self.microbits = self.get_connection(microbits)
        self.vectors = []
        self.gravity_north_angle = None

    @property
    def angles_ref0(self):
        return [
            get_angle([vector[1], vector[2]], [self.vectors[0][1], self.vectors[0][2]])
            for vector in self.vectors
        ]

    @property
    def angles_refn(self):
        return [
            get_angle([vector[1], vector[2]], [self.vectors[n][1], self.vectors[n][2]])
            for n, vector in enumerate(self.vectors[1:])
        ]

    def startup(self):
        for image in (
            Image.CLOCK1,
            Image.CLOCK2,
            Image.CLOCK3,
            Image.CLOCK4,
            Image.CLOCK5,
            Image.CLOCK6,
            Image.CLOCK7,
            Image.CLOCK8,
            Image.CLOCK9,
            Image.CLOCK10,
            Image.CLOCK11,
            Image.CLOCK12,
        ):
            self.arm_mb.led.show(image)
            sleep(1)
        self.gravity_north_angle = get_angle(
            self._get_magnetometer(self.microbits[0]),
            self._get_accelerometer(self.microbits[0]),
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
    def get_connection(microbits: List[KaspersMicrobit | str]):
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
                        f"Invalid type of {microbit}, it should be either KaspersMicrobit or str."
                    )
        return connected_kms

    # Atualiza os vetores para o estado atual

    @staticmethod
    def get_vector(north, fixed_north):
        north_xy = np.array([north[0], north[1]])

        theta = get_angle(north_xy, np.array([0, -1]))
        rtheta = np.array(
            [
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1],
            ]
        )
        north_yz = np.array([north[1], north[2]])
        phi = get_angle(north_yz, np.array([-1, 0]))
        rphi = np.array(
            [
                [1, 0, 0],
                [0, np.cos(phi), -np.sin(phi)],
                [0, np.sin(phi), np.cos(phi)],
            ]
        )
        return np.dot(rphi, np.dot(rtheta, fixed_north))

    def update(self):
        # Define o vetor no sistema de coordenadas do primeiro segmento da
        # articulação como [0,-1,0].
        vectors = []

        # Executa para cada segmento do braço após o primeiro
        for microbit in self.microbits:
            # Define o vetor norte no sistema de coordenadas do microbit deste
            # segmento
            north = self._get_magnetometer(microbit)
            vectors.append(self.get_vector(north, self.fixed_north))

        # Atualiza a lista de vetores dessa instância do objeto JointTracker
        self.vectors = vectors
