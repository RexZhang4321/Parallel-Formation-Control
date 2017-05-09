import matplotlib.pyplot as plt


fp = open('./barrier', 'r')
fig = plt.figure()
ax = fig.add_subplot(111)
x1 = []
x2 = []
y1 = []
y2 = []
for line in fp:
    x, y, cost = line.strip().split('\t')
    if float(cost) == -1.0:
        x1.append(int(x))
        y1.append(int(y))
    else:
        x2.append(int(x))
        y2.append(int(y))
ax.plot(x1, y1, 'r.')
ax.plot(x2, y2, 'g.')
fp.close()

for i in xrange(16):
    with open('./robot' + str(i), 'r') as fp:
        x_arr = []
        y_arr = []
        for line in fp:
            x, y = line.strip().split('\t')
            x_arr.append(float(x))
            y_arr.append(float(y))
        ax.plot(x_arr, y_arr, '.')

plt.show()
