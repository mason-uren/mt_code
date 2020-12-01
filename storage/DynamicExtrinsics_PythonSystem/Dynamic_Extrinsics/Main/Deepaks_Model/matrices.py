import numpy as np

def augment_rotation(rmat):
    return np.block([[
                  rmat,     np.array([[0],[0],[0]])],
        [np.array([0,0,0,    1])]
    ])

def yaw(rads):
    """
    Returns a 3x3 matrix representing a rotation about the y-axis.
    """
    return augment_rotation(np.array([
        [np.cos(rads),      0,     np.sin(rads)],
        [0,                 1,     0           ],
        [-np.sin(rads),     0,     np.cos(rads)]
    ]))
    
def pitch(rads):
    """
    Returns a 3x3 matrix representing a rotation about the x-axis.
    """
    return augment_rotation(np.array([
        [1,   0,              0           ],
        [0,   np.cos(rads),  -np.sin(rads)],
        [0,   np.sin(rads),   np.cos(rads)]
    ]))

def roll(rads):
    """
    Returns a 3x3 matrix representing a rotation about the z-axis.
    """
    return augment_rotation(np.array([
        [np.cos(rads),   -np.sin(rads),    0],
        [np.sin(rads),    np.cos(rads),    0],
        [0,               0,               1]
    ]))
    

def rotation(rx, ry, rz):
    """
    Returns a 4x4 matrix representing a translation in 3d. rx, ry, and rz
    correspond to pitch, yaw, and roll respectively.
    """
    return np.linalg.multi_dot([roll(rz), yaw(ry), pitch(rx)])

def translation(tx, ty, tz):
    """
    Returns a 4x4 matrix representing a translation in 3d.
    """
    tmat = np.eye(4)
    tmat[0:3, 3] = [tx, ty, tz]

    return tmat

# Tester
if __name__ == '__main__':
    print(translation(2,3,4))
    print(yaw(np.pi/2))
    print(pitch(np.pi/2))
    print(roll(np.pi/2))
    print(rotation(np.pi/2,np.pi/2,np.pi/2))
