from time import sleep
from typing import List, Tuple
import time
import copy
from os import system

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
    fixed_gravity = np.array([0, 0, -1])

    class ArticulationState:
        get_speeds = None
        get_accs = None

        def __init__(self, vectors, angles, upd_time, ref_state) -> None:
            self.vectors = vectors
            self.angles = angles
            self.upd_time = upd_time
            self.speeds = JointTracker.ArticulationState.get_speeds(self, ref_state)
            self.accs = JointTracker.ArticulationState.get_accs(self, ref_state)

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

    def __init__(self, *microbits: str | KaspersMicrobit, calculation_time, record):
        self.microbits: List[KaspersMicrobit] = self.get_connection(microbits)
        self.vectors: List[np.ndarray] = []
        self.update = self.start
        self.start_time = None
        self.record_state = self._record_state if record else self.dummy
        self.states: List[self.ArticulationState] = []
        self.latest_states: List[self.ArticulationState] = []
        self.calculation_time = calculation_time
        self.get_state = self.get_first_state
        self.ArticulationState.get_speeds = lambda x, y: [0 for _ in self.microbits]
        self.ArticulationState.get_accs = lambda x, y: [0 for _ in self.microbits]

    @property
    def state(self):
        return self.latest_states[-1]

    def dummy(self, dummyvar):
        pass

    def _record_state(self, state):
        self.states.append(state)

    def update_states(self, state):
        self.vectors = state.vectors
        self.record_state(state)
        for n, _state in enumerate(self.latest_states):
            if state.upd_time - _state.upd_time < self.calculation_time:
                self.latest_states = self.latest_states[n:]
                break
        self.latest_states.append(state)
        system("cls")
        print("vectors: ", state.vectors)
        print("angles (degrees):", [np.degrees(x) for x in state.angles])
        print("angular speeds (rad/s):", state.speeds)
        print("angular accelerations (rad/s²):", state.accs)
        print("elapsed time:", state.upd_time - self.start_time)

    def start(self):
        self.start_time = time.time()
        self.update = self._update
        self.update()

    @staticmethod
    def _get_accelerometer(mb: KaspersMicrobit):
        data = mb.accelerometer.read()
        return np.array([data.x, data.y, data.z])

    @staticmethod
    def _get_magnetometer(mb: KaspersMicrobit):
        data = mb.magnetometer.read_data()
        return np.array([data.x, data.y, data.z])

    @staticmethod
    def get_connection(microbits):
        connected_kms = []
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

    @staticmethod
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

    def get_first_state(self, vectors, angles):
        self.update_states(self.ArticulationState(vectors, angles, time.time(), None))
        self.ArticulationState.get_speeds = self.ArticulationState._get_speeds
        self.ArticulationState.get_accs = self.ArticulationState._get_accs
        self.get_state = self._get_state

    def _get_state(self, vectors, angles):
        self.update_states(
            self.ArticulationState(vectors, angles, time.time(), self.latest_states[0])
        )
        self.ArticulationState.get_speeds = self.ArticulationState._get_speeds
        self.ArticulationState.get_accs = self.ArticulationState._get_accs

    def _update(self):
        vectors = []
        angles = []
        # Executa para cada segmento do braço após o primeiro
        for microbit in self.microbits:
            acc = self._get_accelerometer(microbit)
            vector, angle = self.get_positions(acc, self.fixed_gravity)
            vectors.append(vector)
            angles.append(angle)
        # Atualiza a lista de vetores dessa instância do objeto JointTracker
        self.vectors = vectors
        angles = [angles[0]] + [angle - angles[n] for n, angle in enumerate(angles[1:])]
        self.get_state(vectors, angles)
