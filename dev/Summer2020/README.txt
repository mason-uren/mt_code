Directory containing code for camera intrinsics calibration and fit analysis
Last Updated by Aaron Feldman 9/25/2020

Chauruco_Specific files:
CharucoBoards.py: Specifies different board formats used for calibration.
ChArUcoHelpers.py: Provides helper functions for working with charuco boards including board pose estimation
or reprojection error computation. Includes useful visualization tools like quiver and heat map plots.
generateCharucoBoard.py: Used for creating a Charuco board.


L-Beam files:
detectBeam.py: Used for experimenting with detection parameters to see what gives good detection of the L-beam fiducials
when captured by the imperx.
estimateBeam.py: Detects the two fiducials of the L-beam, estimates the poses given intrinsics knowledge, and computes
the relative pose between the fiducials. Comparing with the ground truth relative pose between the fiducials provides a
way of assessing the success of the intrinsic calibration.
analyzeLbeam.py: See how accuracy/stability of L-beam estimation changes with the number of calibration
training images used. Relies on code found in estimateBeam.py.
LbeamThinLens.py: Implements alternating thin-lens based approach for L-Beam estimation.


Parameter Experimentation files:
detectionParameters.py: Used for experimenting with detection parameters but in more general context than detectBeam.py.
By plotting the number of corners picked up in different images, can choose the appropriate aruco detection parameter 
settings.
parameterTestingIntrinsics.py: Used for experimenting with different distortion models for intrinsics calibration
estimatBeam.py
RANSACdetection.py: Some experimentation with using RANSAC-based solvePnP. The openCV documentation seems
out of date and determining the meaning of some of the arguments was difficult. Ultimately abandoned.


Calibration Fitting and Evaluation files:
intrinsics.py: Used for performing the actual camera intrinsics calibration given an image set. Allows for visualization
of reprojection errors via quiver and heat plots and for different parameters and distortion models.
evaluateIntrinsics.py: Used for easily computing the reprojection error on a dataset given knowledge of the intrinsics.
analyzeFit.py: Provides more thorough analysis of the calibration process through cross validation and learning curves.
evaluatePoint.py: Considers how important varying the calibration parameters is to a good fit.
Looks at the impact of using the point estimate from the wrong focus position and the impact of perturbing
one intrinsics parameter while leaving the rest fixed.
bootstrapPoint.py: Implements a bootstrapping-approach for refined individual (point estimate) calibration.
Repeatedly draw subsets of images and calibrate. Then, repeatedly eliminate outliers until remaining 
intrinsics are all close.
collinearityMetric.py: Given the intrinsics, undistorts the detected chorner points, computes best fit lines for each row
and column and sums the deviation of each chorner from this line as the error. Potentially useful as an alternative
fitting/evaluation metric because it is decoupled from the extrinsics while re-projection is not.

Dynamic Intrinsics Files:
dynamicIntrinsicsHelpers.py: Central file for many functions which are used repeatedly throughout the
other dynamic intrinsics files.
fitFixedModels.py: Similar to intrinsics.py, but computes camera intrinsics for a series of different focus positions
that are assumed to be neighboring subfolders.
fitVaryingModel.py: Given the point estimated (fixed) camera intrinsics at different focus positions, uses polynomial fitting 
(or moving least squares) to fit a varying/generalized camera model wrt focus position. Also includes assessment approaches 
like leave-one-out cross validation relative to the point estimates or for the L-beam.
(Failed) altmin.py: Used for implementing the alternating gradient-based descent given an initial polynomial fit for the 
varying model. After learning more about the system, seems like won't work well because intrinsics and extrinsics are tightly
coupled. Instead, using focusBundleAdjustment.py would work better.
hartleyUndistort.py: Implements Parameter-Free Radial Distortion Correction with Center of Distortion Estimation. Only the
first portion which tries to find the center of distortion works and I suspect the second portion which works on estimating
the actual distortion has a bug. Even so, having an estimate of the center of distortion, which is oftentimes assumed to be
the optical center could be valuable.
Mexperiments.py: Wanted to see how scaling or rotating the board frame (without actually changing the images) would impact the
resulting magnification M, ratio of Z and effective focal length in world units. Seems like increasing the square size by a 
factor will increase M by that same factor and that rotating the board frame does not impact M. Would also be interesting to
look at how M is impacted by shift in the X, Y camera plane.
rotTransExperiments.py: Look at how M changes in practice when performing rotation about the origin or translating in the X,Y
camera plane. Obviously, hard to perfectly control such motions perfectly but can still get an idea. Compares the estimates
using fixed intrinsics and the alternating thin lens approach.
singleBoardComparisons.py: Compares point, fixed, and thin lens approaches for estimating a single board. Also, used to better
understand how well the theory holds for the difference between the fixed and thin lens approaches.
syntheticImages.py: Started working on generating synthetic images given the intrinsics. Currently, does not work with
distortion but can incorporate rotation and translation.
theoryTest.py: Looks at how changing effective focal length impacts the pose estimates and compares this to what would
theoretically predict from a thin lens (and pinhole for some of the analysis) model.
postItExperiment.py: Used for analyzing and comparing different approaches in the known Z-shift experiments. Move camera back
a known amount so that can compare the measured and estimated change.
MbundleAdjustment.py: Assume that dynamic intrinsics vary according to the thin lens model, that the center is linear
or constant wrt focus motor position, and that distortion is linear with magnification. Then, fit the model by minimizing
reprojection error across a dataset, alternatingly computing updated magnification.
focusBundleAdjustment.py: Assume that dynamic intrinsics vary according to focus and fit the model by minimizing reprojection
error across a dataset. Can be used to refine an initial polynomial fit which treats each parameter independently.
thinLensAgainstPoint.py: Used for comparing the estimates made by the alternating thin lens approach against the point estimates
while comparing them against absolute change in Z. Also, seeing how well Z can be fit to the thin lens predicted fM + f 
for some f.
fitAgainstPoint.py: Used for comparing the estimates made by the polynomial fit approach against the alternating thin lens
and against absolute Z.

File Management:
saveDeltas.py: Used for saving to an excel table the deviation of the various models from the point estimates.
preparePostIt.py: Used for converting a set of folders each containing one image at a different focus position into one
folder with all the images.




