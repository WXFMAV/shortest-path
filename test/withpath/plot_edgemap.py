import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('edgemap_out.txt')

plt.figure(num = 1, figsize = (8,6))
plt.title('edgemap')
plt.xlabel('x axis', size = 20)
plt.ylabel('y axis', size = 20)

#print data[0,0], data[0,1], data[0,2],data[0,3], data[0,4],data[0,5]
#print data[1,0]

for index in range(0, len(data[:,0])):
#    print index
    xdata = np.array([data[index, 0], data[index, 2], data[index, 4]])
    ydata = np.array([data[index, 1], data[index, 3], data[index, 5]])
#    print xdata
#    print ydata
    plt.plot(xdata, ydata, color = 'b', linestyle = '--', marker = '.')

path = np.loadtxt('edgepath_out.txt')
xdata = path[:,0]
ydata = path[:,1]
plt.plot(xdata + 10.0, ydata + 10.0, color = 'r', linewidth = 4.0, linestyle = '-', marker = '.')

plt.savefig('plot_edgemap.png', format = 'png')

