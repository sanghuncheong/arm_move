a = [[0, 1], [1, 2], [1], [3, 2, 1],[]]
al = []
for i in range(len(a)):
    al.append(len(a[i]))
print a
print al
print al.index(min(al))
print "min a len a = ", a[al.index(min(al))]