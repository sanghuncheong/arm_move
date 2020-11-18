a = [0, [1], [1,2,3],[1,2,4],[],1,[]]

print a
# for i in range(10):
cnt = 0
for i in a:
    if i == []:
        cnt = cnt + 1
        print i,": []"
for i in range(cnt):
    a.remove([])
print a

print min([1])