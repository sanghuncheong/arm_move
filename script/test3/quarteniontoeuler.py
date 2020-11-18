from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math

angle = quaternion_from_euler(math.radians(90), math.radians(-90), 0, axes='rxyz')
print angle