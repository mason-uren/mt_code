# A script to plot 3D trajectories saved by the Fiducial Tracker by the name of
# 'FiducialTracker[ 0]_charuco_poses.txt' and 'FiducialTracker[ 1]_charuco_poses.txt'
# The input files must be changed first to run this script.

import pandas as pd
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np 

#Ref: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to

def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


#track0=pd.read_csv('FiducialTracker[ 0]_charuco_poses.txt', sep=' ', names=['frameid','x','y','z','pitch','yaw','roll','reproj_err'])
#track1=pd.read_csv('FiducialTracker[ 1]_charuco_poses.txt', sep=' ', names=['frameid','x','y','z','pitch','yaw','roll','reproj_err'])
#track0=pd.read_csv('data/Expr1_tracker0_poses_scale0.15.txt', sep=' ', names=['frameid','x','y','z','pitch','yaw','roll','reproj_err'])
#track1=pd.read_csv('data/Expr1_tracker0_poses_scale0.2.txt', sep=' ', names=['frameid','x','y','z','pitch','yaw','roll','reproj_err'])
track2=pd.read_csv('data/Expr2_tracker0_poses_scale0.15.txt', sep=' ', names=['frameid','x','y','z','pitch','yaw','roll','reproj_err'])
track3=pd.read_csv('data/Expr2_tracker1_poses_scale0.15.txt', sep=' ', names=['frameid','x','y','z','pitch','yaw','roll','reproj_err'])

#print(track)

fig = plt.figure('Tracked Fiducial Trajectory')
ax = plt.axes(projection='3d')
#ax.set_aspect('equal') # does not work

#ax.plot(track0.x,track0.y,track0.z, zdir='z');
#ax.plot(track1.x,track1.y,track1.z);
ax.plot(track2.x,track2.y,track2.z);
ax.plot(track3.x,track3.y,track3.z);

# Uncomment the following line to plot with equal axes (equal scales in all axes)
#set_axes_equal(ax)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
#plt.axis('equal')
#plt.legend(['Image scale 0.15 (original)','Image scale 0.20 (improved)'])
#plt.zlabel('Z')

plt.show()
