import pandas as pd
from recording import record_microbits, Characteristic
from kaspersmicrobit import KaspersMicrobit
import numpy as np


def microbit_data_to_dataframe(microbit_data: dict[Characteristic, np.ndarray]):
    data_dict = {}
    for characteristic, data in microbit_data.items():
        if characteristic == Characteristic.ACCELEROMETER or characteristic == Characteristic.MAGNETOMETER:
            print(data)
            data_dict[f'{characteristic.name}_X'] = data[:, 0]
            data_dict[f'{characteristic.name}_Y'] = data[:, 1]
            data_dict[f'{characteristic.name}_Z'] = data[:, 2]
        elif characteristic == Characteristic.IOPIN:
            for i in range(data.shape[1]):
                data_dict[f'{characteristic.name}_{i}'] = data[:, i]
        elif characteristic == 'TIME':
            data_dict['TIME'] = data
        else:
            data_dict[characteristic.name] = data
    return pd.DataFrame(data_dict)


if __name__ == '__main__':
    # Microbit 0
    ADDRESS = 'C3:B0:42:88:FE:07'
    # Microbit 1
    ADDRESS1 = 'F6:8C:51:58:97:63'
    microbit_0 = KaspersMicrobit(ADDRESS)
    microbit_1 = KaspersMicrobit(ADDRESS1)
    microbit_0.connect()
    # microbit_1.connect()
    microbit_0.magnetometer.calibrate()
    result = record_microbits(microbit_0,
                              characteristics=[Characteristic.MAGNETOMETER],
                              time_length=300, verbose=True)
    for n, data in enumerate(result):
        microbit_data_to_dataframe(data).to_excel(
            'microbit_data_0C.xlsx', index=False)
