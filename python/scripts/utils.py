import matplotlib.pyplot as plt
import numpy as np
import json 

folder    = "../../tests/json_files/my_folder/"
file_name = "barrier_1.json"

## read the json file
with open(folder + file_name, 'r') as f:
    data = json.load(f)

print(data)
slopes    = np.array(data['Slopes'])
slack     = np.array(data['Slack'])
gamma0    = np.array(data['Gamma0'])
r         = np.array(data['R'])
b         = np.array(data['b_vector'])
time_grid = np.array(data['TimeGrid'])


if len(gamma0) == 4 :
    slopes = slopes.reshape(4,-1)
elif len(gamma0) == 6 :
    slopes = slopes.reshape(6,-1)

print("Slopes shape: ", slopes.shape)
print("Slack shape: " , slack.shape)
print("Gamma0 shape: ", gamma0.shape)
print("R shape: "     , r.shape)


def compute_offset_vector(slopes, gamma0, t, time_grid, b_vector):
    """
    Compute the offset vector based on slopes, gamma0, and r.
    """
    tt            = np.zeros(slopes.shape[1])
    delta_t       = np.diff(time_grid)
    idx           = np.searchsorted(time_grid, t, side='left')-1
    
    if idx < 0:
        tt[0] = t-time_grid[0]

    else:
        for ii in range(idx):
            tt[ii] = delta_t[ii]

        tt[idx] = t-time_grid[idx]


    return slopes @ tt + gamma0 + b_vector
    



# compute_offset_vector(slopes, gamma0, 4, time_grid, b)
fig, ax = plt.subplots(figsize=(10, 6))

B = np.zeros((100,slopes.shape[0]))
for jj,t in enumerate(np.linspace(0,time_grid[-1], 100)):
    b_vector = compute_offset_vector(slopes, gamma0, t, time_grid, b)
    B[jj,:] = b_vector

# plot 
ax.plot(np.linspace(0,time_grid[-1], 100), B)


fig, ax = plt.subplots(figsize=(10, 6))

## create a scatter plot
for jj,t in enumerate(np.linspace(0,time_grid[-1], 100)):
    b_vector = compute_offset_vector(slopes, gamma0, t, time_grid, b)

    x_max = b_vector[0]
    x_min = -b_vector[1]
    y_max = b_vector[2]
    y_min = -b_vector[3]

    vertices = np.array([[x_max, y_max], [x_min, y_max], [x_min, y_min], [x_max, y_min], [x_max, y_max]])
    # create a rectangle patch 
    ax.scatter(vertices[:, 0], vertices[:, 1], color='black', alpha=1*jj/100, linewidth=1.5)  
    ax.plot(vertices[:, 0], vertices[:, 1], color='black', alpha=0.5, linewidth=1.5)  






plt.show()

