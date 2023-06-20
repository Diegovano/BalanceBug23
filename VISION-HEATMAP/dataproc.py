import pandas as pd
import math

measured_data = pd.read_csv('real_data.csv')

# measured_data.head()

measured_points = [r[0:2] for r in measured_data.to_numpy()]
resec_points = [r[2:4] for r in measured_data.to_numpy()]

def dist(point1, point2):
  return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
  
errors = []
avg_err = 0

for i, mp in enumerate(measured_points):
  errors.append(dist(mp, resec_points[i]))
  avg_err += errors[i]

avg_err /= len(errors)
max_err = max(errors)
min_err = min(errors)

err_range = max(max_err - avg_err, avg_err - min_err)
# norm_err = [ (((e - avg_err)/ err_range) / 2) + 0.5 for e in errors]

norm_err = [ e / max_err for e in errors]

print(avg_err, max_err, min_err)

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

viridis = mpl.colormaps['RdYlGn_r'].resampled(8)

fig, axs = plt.subplots(1,2)
ax = axs[0]

ax.scatter([r[0] for r in measured_points], [r[1] for r in measured_points], label='real position')
ax.scatter([r[0] for r in resec_points], [r[1] for r in resec_points], label='triangulated')

# connects all of the dots together
# ax.plot([r[0] for r in measured_points], [r[1] for r in measured_points],[r[0] for r in resec_points], [r[1] for r in resec_points])

for count, (mx, my) in enumerate(measured_points):
  rx, ry =  resec_points[count]
  ax.plot([mx, rx], [my, ry], c = viridis(norm_err[count]), linewidth=2, zorder=0)

# draw gridlines
# ax.grid(which='major', axis='both', linestyle='dotted', color='lightgray', linewidth=0.7)
ax.set_xticks(np.arange(0, 241, 60))
ax.set_yticks(np.arange(0, 360, 60))

ax.set_xlabel('x')
ax.set_ylabel('y')

ax.set_aspect(1)
ax.grid(True)

ax.legend()
# fig.colorbar(ax)

ax1 = axs[1]
x = np.arange(30, 240, 60) # 4
y = np.arange(30, 360, 60) # 6
z = np.zeros((6,4))

for count, (mx, my) in enumerate(measured_points):
  x_ind = math.floor(mx / 60)
  y_ind = math.floor(my / 60)
  # print(x_ind, y_ind, mx, my)
  z[y_ind,x_ind] = errors[count]

ax1.set_xticks(np.arange(0, 241, 60))
ax1.set_yticks(np.arange(0, 360, 60))

ax1.set_xlabel('x')
ax1.set_ylabel('y')

ax1.set_aspect(1)
# ax1.grid(True)
psm = ax1.pcolormesh(x, y, z, cmap='RdYlGn_r')
fig.colorbar(psm, label='cm from correct position')


plt.show()