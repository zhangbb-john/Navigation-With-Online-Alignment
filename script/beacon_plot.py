import matplotlib.pyplot as plt 
import numpy as np

data = np.loadtxt('/home/ubuntu/Bearing-only/log/beacon.txt')


fig = plt.figure()
ax = fig.add_subplot(231)
ax.scatter(data[:, 0], data[:, 1], marker='o', color='r')
ax.set_xlabel('east')
ax.set_ylabel('north')
ax.set_title('odom+usbl = beacon position')

plt.show()