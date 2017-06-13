import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('bazier_out.txt')   
xData = data[:,0]
yData1 = data[:,1]

plt.figure(num=1, figsize=(8, 6))
plt.title('Plot 1', size=14)
plt.xlabel('x-axis', size=14)
plt.ylabel('y-axis', size=14)
plt.plot(xData, yData1, color='b', linestyle='--', marker='o', label='y1 data')
plt.legend(loc='upper left')
plt.savefig('plot_bazier.png', format='png')
