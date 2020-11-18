a = "Found a contact between 'can' (type 'Object') and 'Body_RWY' (type 'Robot link'), which constitutes a collision. Contact information is not stored."

print len(a)

b = a.split("'")
for i in range(len(b)):
    print i, b[i]

print "\n collision:", b[1], b[5]