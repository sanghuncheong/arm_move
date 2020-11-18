import numpy as np

a = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

for i in range(len(a)):
    print "i", i
    for j in range(10):
        print "i, j", i, j
        if j == 3:
            break