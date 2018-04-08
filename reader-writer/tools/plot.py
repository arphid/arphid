from pylab import *
import sys


data = []
for line in sys.stdin:
	data.append([])
	for field in line.split():
		data[-1].append(float(field))


for i, line in enumerate(zip(*data)):
	plot(line, label=str(i))
legend(loc='lower right')
grid(True)
show()
