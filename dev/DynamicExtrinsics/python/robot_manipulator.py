
# This file defines a robot manipulator (e.g., a tooltip, a printer head, etc.) to which
# a ChAruCo fiducial is affixed as a proxy for accurate pose measurement by the multi-camera
# based postioning system being developed under Boeing DR Closed-Loop Metrology project.

import numpy as np

# Simplest configuration of this relationship is for the robot manipulator to be
# represented by a point, relative to the fiducial board. More specifically, we
# will assume this point to be located behind the fiducial board (away from the camera)
# and at a distance D from the center of the fiducial board (we assume the fiducial
# board has its Z axis facing the camera to be visible, and x & y in the board surface).

D = -0.25 # meters
robot_pt_ext = np.array([ [ 1.0, 0.0, 0.0, 0.0 ],
						  [ 0.0, 1.0, 0.0, 0.0 ],
						  [ 0.0, 0.0, 1.0, D ],
						  [ 0.0, 0.0, 0.0, 1.0],
						])
