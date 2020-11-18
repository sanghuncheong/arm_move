def object_orientation(obstacle_position, obstacle_orientation):

    import numpy as np

    x1 = 0.1 * np.cos(obstacle_orientation[2]) + obstacle_position[0]
    y1 = 0.1 * np.sin(obstacle_orientation[2]) + obstacle_position[1]

    x2 = -0.1 * np.cos(obstacle_orientation[2]) + obstacle_position[0]
    y2 = -0.1 * np.sin(obstacle_orientation[2]) + obstacle_position[1]

    #print([x1, y1], [x2, y2])

    return [x1, y1], [x2, y2]