def linspace2D(start, stop, num=50, endpoint=True, retstep=False, dtype=None):
    import numpy as np
    ret = []
    px = np.linspace(start[0], stop[0], num=10)
    py = np.linspace(start[1], stop[1], num=10)
    for i in zip(px, py):
        ret.append(list(i))
    return ret

