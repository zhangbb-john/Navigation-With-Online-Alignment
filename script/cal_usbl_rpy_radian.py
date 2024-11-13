import numpy as np
scale = input('input the scale: \n')
angle_deg = np.array([1, 2, 3])
angle_rad = angle_deg / 180.0 * np.pi
print(scale * angle_rad)