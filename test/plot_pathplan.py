import numpy as np
from matplotlib.patches import Ellipse, Circle
import matplotlib.pyplot as plt


plt.figure(num = 1, figsize = (8,6))
plt.title('edgemap')
plt.xlabel('x axis', size = 20)
plt.ylabel('y axis', size = 20)

#print data[0,0], data[0,1], data[0,2],data[0,3], data[0,4],data[0,5]
#print data[1,0]
#obs: 2.45234 -4.35515
#obs: 4.36328 2.45407
#obs: -2.45079 4.35211
#obs: -4.35216 -2.44983

cir1 = Circle(xy = (1.0 + 10, 1.0 + 10), radius=2, alpha=0.5)
plt.figure().add_subplot(111).add_patch(cir1);

path = np.loadtxt('edgepath_out.txt')
xdata = path[:,0]
ydata = path[:,1]
plt.plot(xdata + 10.0, ydata + 10.0, color = 'r', linestyle = '-', marker = '.')

plt.plot(0,0);
plt.plot(0,20);
plt.plot(20,0);
plt.plot(20,20);


plt.savefig('plot_edgemap.png', format = 'png')

