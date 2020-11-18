
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    import numpy as np
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    import numpy as np
    import math
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def get_approaching_pose(cp, ap, gp):
    import numpy as np
    from tf.transformations import quaternion_from_euler

    l_x = np.linalg.norm(gp - cp)   # size of vector x
    l_z = np.linalg.norm(ap - cp)   # size of vector z
    # print "dx, dz", l_x, l_z
    u_x = (gp - cp) / l_x   # unit vector x from the given data
    u_z = (ap - cp) / l_z   # unit vector z from the given data
    u_y = np.cross(u_z, u_x)    # compute unit vector y from crossing the unit vector z to x
    # print u_x, u_y, u_z
    rot_mat = [u_x, u_y, u_z]
    # print "rotation matrix:\n", rot_mat
    # print "transposed:\n", np.transpose(rot_mat)
    ret = rotationMatrixToEulerAngles(np.transpose(rot_mat))    # compute the euler angle from the rotation matrix
    # print "return:", ret
    ans = quaternion_from_euler(ret[0], ret[1], ret[2], 'ryxz') # get quaternion from euler angle
    # print "answer:", ans
    t_pos = list(ap)
    t_ori = list(ans)
    t_pos.extend(t_ori)
    return t_pos    # [x, y, z, qx, qy, qz, qw]

if __name__ == '__main__':
    import numpy as np
    # case up
    # center point
    cp_x = 0
    cp_y = 0
    cp_z = 0
    cp = np.array([cp_x, cp_y, cp_z])

    # z-axes
    ap_x = 0
    ap_y = 0
    ap_z = 2
    ap = np.array([ap_x, ap_y, ap_z])

    # x-axes
    gp_x = 0
    gp_y = 2
    gp_z = 0
    gp = np.array([gp_x, gp_y, gp_z])

    ret = get_approaching_pose(cp, ap, gp)
    print "return vector:", ret