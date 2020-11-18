import copy

t_r = 0.5
o_r = [0.1, 0.2, 0.3]

R = copy.deepcopy(o_r)
R.append(t_r)

print "R:", R

N = 4
H = []
for i in range(N):
    H.append(0.075)

print "H:", H

o_p = [[0.1, 0.1], [0.2, 0.2], [0.3, 0.3]]
t_p = [0.5, 0.5]

X = []
Y = []

for i in range(len(o_p)):
    X.append(o_p[i][0])
    Y.append(o_p[i][1])

X.append(t_p[0])
Y.append(t_p[1])

print "X:", X
print "Y:", Y

zero = [0.0, 0.0]
wd = [0.5, 0.8]


x_min = zero[0]
x_max = zero[0] + wd[0]
y_min = zero[1] - wd[1]
y_max = zero[1]