
for n_obs_max in range(1, 15+1):
    for test_i in range(10):
        obs = []
        for n_obs in range(n_obs_max):
            obs.append(1)
        print "test i:", test_i, "n obs:", len(obs)