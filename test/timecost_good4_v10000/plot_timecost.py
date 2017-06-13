import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('timecost_out.txt')

plt.figure(num = 1, figsize = (8,6))
plt.title('timecost')
plt.xlabel('x axis', size = 20)
plt.ylabel('y axis', size = 20)

xdata = data[:,0]
ydata = data[:,2]
plt.plot(xdata , ydata , color = 'r', linestyle = '-', marker = '.')
xdata = data[:,0]
ydata = data[:,4]
plt.plot(xdata , ydata , color = 'r', linestyle = '-', marker = '.')
plt.savefig('plot_timecost.png', format = 'png')

