import numpy as np

obs = [[0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6]]

poss_obs = [0, 1, 3]

for po in poss_obs:
    print "\nwork with rearranging additional ", obs[po]

for po1 in poss_obs:
    poss_obs2 = [2, 4, 5]
    for po2 in poss_obs2:
        print "\nwork with rearranging additional ", obs[po1]
        print "work with rearranging additional ", obs[po2]

