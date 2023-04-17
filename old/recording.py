from enum import IntEnum
import time
from typing import Dict, Iterable
import numpy as np
from kaspersmicrobit import KaspersMicrobit
from mb_angles import get_angles
from os import system


class Characteristic(IntEnum):
    """
    The characteristics that can be recorded.
    """
    ACCELEROMETER = 0
    BUTTONA = 1
    BUTTONB = 2
    TEMPERATURE = 3
    IOPIN = 4
    LED = 5
    MAGNETOMETER = 6


class MicrobitRecorder:
    def __init__(self, microbit: KaspersMicrobit, characteristics):
        self.microbit = microbit
        self.data: Dict = dict()
        if Characteristic.ACCELEROMETER in characteristics:
            self.data[Characteristic.ACCELEROMETER] = []
            self.update_acellerometer = self.actually_update_acellerometer
        else:
            self.update_acellerometer = self.dummy_update
        if Characteristic.BUTTONA in characteristics:
            self.data[Characteristic.BUTTONA] = []
            self.update_button_a = self.actually_update_button_a
        else:
            self.update_button_a = self.dummy_update
        if Characteristic.BUTTONB in characteristics:
            self.data[Characteristic.BUTTONB] = []
            self.update_button_b = self.actually_update_button_b
        else:
            self.update_button_b = self.dummy_update
        if Characteristic.TEMPERATURE in characteristics:
            self.data[Characteristic.TEMPERATURE] = []
            self.update_temperature = self.actually_update_temperature
        else:
            self.update_temperature = self.dummy_update
        if Characteristic.IOPIN in characteristics:
            num_inputs = str(self.microbit.io_pin.read_io_configuration()).\
                count('PinIO.INPUT')
            if num_inputs > 0:
                self.data[Characteristic.IOPIN] = []
                self.update_io_pin = self.actually_update_io_pin
            else:
                self.update_io_pin = self.dummy_update
        else:
            self.update_io_pin = self.dummy_update
        if Characteristic.LED in characteristics:
            self.data[Characteristic.LED] = []
            self.update_led = self.actually_update_led
        else:
            self.update_led = self.dummy_update
        if Characteristic.MAGNETOMETER in characteristics:
            self.data[Characteristic.MAGNETOMETER] = []
            self.update_magnetometer = self.actually_update_magnetometer
        else:
            self.update_magnetometer = self.dummy_update

    def update(self):
        self.update_acellerometer()
        self.update_button_a()
        self.update_button_b()
        self.update_temperature()
        self.update_io_pin()
        self.update_led()
        self.update_magnetometer()

    def dummy_update(self):
        pass

    def actually_update_acellerometer(self):
        acc_data = self.microbit.accelerometer.read()
        self.data[Characteristic.ACCELEROMETER].append(
            [acc_data.x, acc_data.y, acc_data.z])

    def actually_update_button_a(self):
        self.data[Characteristic.BUTTONA].append(
            self.microbit.buttons.read_button_a())

    def actually_update_button_b(self):
        self.data[Characteristic.BUTTONB].append(
            self.microbit.buttons.read_button_b())

    def actually_update_temperature(self):
        self.data[Characteristic.TEMPERATURE].append(
            self.microbit.temperature.read())

    def actually_update_io_pin(self):
        self.data[Characteristic.IOPIN].append(
            self.microbit.io_pin.read_data())

    def actually_update_led(self):
        self.data[Characteristic.LED].append(str(self.microbit.led.read()))

    def actually_update_magnetometer(self):
        mag_data = self.microbit.magnetometer.read_data()
        self.data[Characteristic.MAGNETOMETER].append(
            [mag_data.x, mag_data.y, mag_data.z])

    def print_last_data(self):
        data_string = ""
        for characteristic, data in self.data.items():
            data_string += f"\n\t{characteristic.name}: {data[-1]}"
        data_string += '\n\n'
        return data_string


def record_microbits(*microbits, characteristics: Iterable[Characteristic],
                     time_length: float | int, verbose: bool):
    """
    Record data from the microbits for a given time.
    """
    characteristics = set(
        [characteristic for characteristic in characteristics])
    recorders = [MicrobitRecorder(microbit, characteristics)
                 for microbit in microbits]
    if verbose:
        verbose_fn = print
    else:
        def verbose_fn(*args, **kwargs): pass
    time_list = []
    start_time = time.time()
    now = time.time()
    while now - start_time < time_length:
        time_list.append(now)
        for i, recorder in enumerate(recorders):
            recorder.update()
            verbose_fn(
                f'microbit {i}\t{round(((now-start_time)/time_length)*100, 2)}%', '\n', recorder.print_last_data())
        system('cls')
        verbose_fn(format_data(recorder))
        now = time.time()
    for recorder in recorders:
        recorder.data['TIME'] = time_list
        for characteristic in recorder.data.keys():
            recorder.data[characteristic] = np.array(
                recorder.data[characteristic])
            if characteristic == 'TIME':
                recorder.data[characteristic] = \
                    recorder.data[characteristic] - \
                    start_time - (time_list[0] - start_time)
    return (recorder.data for recorder in recorders)


def angle(v1, v2, acute: bool = True):
    # v1 is your firsr vector
    # v2 is your second vector
    angle = np.arccos(
        np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    if acute:
        return angle * 180 / np.pi
    return (2 * np.pi - angle) * 180 / np.pi


def format_data(recorder):
    return f'MAGNETOMETER: {np.linalg.norm(recorder.data[Characteristic.MAGNETOMETER][-1])}\n' + \
        f'ANGLE_X_MAG: {angle(recorder.data[Characteristic.MAGNETOMETER][-1], [1, 0, 0])}\n' +\
        f'ANGLE_Y_MAG: {angle(recorder.data[Characteristic.MAGNETOMETER][-1], [0, 1, 0])}\n' +\
        f'ANGLE_Z_MAG: {angle(recorder.data[Characteristic.MAGNETOMETER][-1], [0, 0, 1])}\n'
