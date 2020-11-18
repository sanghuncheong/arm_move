import numpy as np

pi = np.pi
r = 0.035
s = pi*r**2

ws = 0.4*0.8
print "work space: ", ws

for i in range(15):
    print "object", i, "space: ", s*i, "rate space: ", s*i*100/ws, "%"
