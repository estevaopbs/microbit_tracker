import pandas as pd
import plotly.express as px
import numpy as np
import random


sample_rate = 0.5
data = pd.read_excel('microbit_data_0.xlsx', 0, usecols="A:C")
vecs = []
for row in data.iterrows():
    vec = np.array([row[1][0], row[1][1], row[1][2]])
    vec = vec / np.linalg.norm(vec)
    vecs.append(vec)
vecs = np.array(vecs)
data = pd.DataFrame({'x': vecs[:, 0], 'y': vecs[:, 1], 'z': vecs[:, 2]})
fig = px.scatter_3d(data, x='x', y='y', z='z')
fig.show()
