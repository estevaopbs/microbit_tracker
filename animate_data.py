from pathlib import Path
from typing import List
import numpy as np
import pandas as pd


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


class RecordedData:
    fixed_north = np.array([0, 1, 0])
    fixed_gravity = np.array([0, 0, -1])

    class ArticulationState:
        get_speeds = None
        get_accs = None

        def __init__(self, vectors, angles, upd_time, ref_state) -> None:
            self.vectors = vectors
            self.angles = angles
            self.upd_time = upd_time
            self.speeds = RecordedData.ArticulationState.get_speeds(self, ref_state)
            self.accs = RecordedData.ArticulationState.get_accs(self, ref_state)

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

    def __init__(self, *microbits_data_path, calculation_time, record):
        self.microbits_data = [pd.read_excel(x) for x in microbits_data_path]
        self.vectors = []
        self.update = self.start
        self.start_time = None
        self.record_state = self._record_state if record else self.dummy
        self.states = []
        self.latest_states = []
        self.calculation_time = calculation_time
        self.get_state = self.get_first_state
        self.row = 0
        self.ArticulationState.get_speeds = lambda x, y: [
            0 for _ in self.microbits_data
        ]
        self.ArticulationState.get_accs = lambda x, y: [0 for _ in self.microbits_data]
        self.last_angles = None

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

    def start(self):
        self.start_time = self.microbits_data[0].iloc[self.row]["TIME"]
        self.update = self._update
        self.update()

    @staticmethod
    def _get_accelerometer(mb, row):
        data = np.array(mb.iloc[row][:3].values)
        return data

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
        self.update_states(
            self.ArticulationState(
                vectors, angles, self.microbits_data[0].iloc[self.row]["TIME"], None
            )
        )
        self.ArticulationState.get_speeds = self.ArticulationState._get_speeds
        self.ArticulationState.get_accs = self.ArticulationState._get_accs
        self.get_state = self._get_state

    def _get_state(self, vectors, angles):
        self.update_states(
            self.ArticulationState(
                vectors,
                angles,
                self.microbits_data[0].iloc[self.row]["TIME"],
                self.latest_states[0],
            )
        )
        self.ArticulationState.get_speeds = self.ArticulationState._get_speeds
        self.ArticulationState.get_accs = self.ArticulationState._get_accs

    def _update(self):
        vectors = []
        angles = []
        # Executa para cada segmento do braço após o primeiro
        for microbit in self.microbits_data:
            acc = self._get_accelerometer(microbit, self.row)
            vector, angle = self.get_positions(acc, self.fixed_gravity)
            vectors.append(vector)
            angles.append(angle)
        # Atualiza a lista de vetores dessa instância do objeto JointTracker
        self.vectors = vectors
        angles = [angles[0]] + [angle - angles[n] for n, angle in enumerate(angles[1:])]
        corrected_angles = []
        if self.last_angles is not None:
            for angle, last_angle in zip(angles, self.last_angles):
                if np.degrees(angle) > 200 and np.degrees(last_angle) < 90:
                    corrected_angles.append(2 * np.pi - angle)
                elif np.degrees(angle) > 200 and np.degrees(last_angle) < 0:
                    corrected_angles.append(2 * np.pi - angle)
                elif np.degrees(angle) < -200 and np.degrees(last_angle) > 0:
                    corrected_angles.append(2 * np.pi + angle)
                else:
                    corrected_angles.append(angle)
            angles = corrected_angles
        self.last_angles = angles
        self.get_state(vectors, angles)
        self.row += 1