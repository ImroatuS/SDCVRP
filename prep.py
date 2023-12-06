import pandas as pd
import numpy as np
import math


# Read the CSV files
df = pd.read_csv('distance_matrix_real.csv')
df2 = pd.read_csv('tps.csv')
df3 = pd.read_csv('kendaraan.csv')


num_vehicles = len(df3)
capacity = df3['capacities'].mean().astype(int)

# Demands per week
df2['demands'] = df2['demands'] / 31 * 28 / 4

result = []
for _, row in df2.iterrows():
    real_id = row['id']
    name = row['name']
    demands = row['demands']
    # letter = 'A'  if demands > capacity else ''

    while demands > capacity:
        result.append((real_id, f'{name}', capacity))
        demands -= capacity
        # letter = chr(ord(letter) + 1)
    result.append((real_id, f'{name}', round(demands)))

data_split = pd.DataFrame(result, columns=['real_id', 'name', 'demands'])
data_split['id'] = data_split.reset_index().index
data_split = data_split[['id', 'real_id', 'name', 'demands']]

# Save to CSV file
data_split.to_csv('data_split.csv', index=False)

names = data_split['real_id'].tolist()
node_id = pd.read_csv('tps.csv')
ids = node_id['id'].tolist()

# Create duplicate data distance matrix
df = df.iloc[:, 1:]
distance_matrix = df.values.tolist()
dfx = pd.DataFrame(distance_matrix, columns=ids)
duplicated_dfx = dfx[names]
duplicated_rows = duplicated_dfx.loc[names]
duplicated_dfx = pd.concat([duplicated_rows], ignore_index=True)

# Create duplicate data demands
demands_split = data_split[['demands']].copy()

num_routes = math.ceil(df2['demands'].sum()/capacity)
vehicle_capacities = np.array([capacity] * num_routes)

distance_matrix = duplicated_dfx.values.tolist()
demands = [item[0] for item in demands_split.values.tolist()]
vehicle_capacities = vehicle_capacities.tolist()

print(f"Distance Matrix: {len(distance_matrix)} x {len(distance_matrix)}")
print(f"Node: {len(demands)}")
print(f"Capacity: {capacity}")
print(f"Vehicle: {num_vehicles}")