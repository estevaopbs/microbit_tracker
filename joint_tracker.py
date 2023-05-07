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
    def __init__(self, *microbits: str | KaspersMicrobit):
        self.microbits = self.get_connection(microbits)
        self.vectors = []
        self.gravity_north_angle = None
        self.angles_ref0 = []
        self.angles_refn = []

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
    def update(self):
        # Define o vetor do primeiro segmento do braço no seu próprio sistema
        # de coordenadas como [0,-1,0].
        vectors = [np.array([0, -1, 0])]
        north_diffs = []
        angles_ref0 = []
        angles_refn = []

        # Define o vetor norte do primeiro segmento
        north0 = self._get_magnetometer(self.microbits[0])

        # Projeta o vetor norte do primeiro segmento no plano yz (plano do
        # braço)
        north0_yz = np.array([north0[1], north0[2]])

        # Normaliza a projeção do vetor norte no plano yz
        north0_yz = north0_yz / np.linalg.norm(north0_yz)

        # Encontra o ângulo entre a projeção do vetor norte primeiro segmento
        # no plano yz
        north_diffs.append(get_angle(np.array([-1, 0]), north0_yz))

        # Executa para cada segmento do braço após o primeiro
        for microbit in self.microbits[1:]:
            # Define o vetor norte no sistema de coordenadas do microbit deste
            # segmento
            northn = self._get_magnetometer(microbit)

            # Projeta o vetor norte no plano yz no sistema de coordenadas do
            # microbit deste segmento
            northn_yz = np.array([northn[1], northn[2]])

            north_diffs.append(get_angle(np.array([-1, 0]), northn_yz))

            angles_ref0.append(2 * np.pi + (north_diffs[-2] - north_diffs[-1]))

            angles_refn.append(2 * np.pi + (north_diffs[-2] - north_diffs[-1]))

            vectors.append(
                np.array([0, -np.cos(angles_ref0[-1]), np.sin(angles_ref0[-1])])
            )

        # Atualiza a lista de vetores dessa instância do objeto JointTracker
        self.vectors = np.array(vectors)
        self.angles_ref0 = np.array(angles_ref0)
        self.angles_refn = np.array(angles_refn)

        # Printa os angulos no terminal
        system("cls")
        print([np.degrees(x) for x in self.angles_refn])

        # Daqui pra baixo é apenas um sonho distante
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
