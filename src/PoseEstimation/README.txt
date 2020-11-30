File: PoseEstimation/README.txt

PoseEstimation module supports fiducial tracking (TrackingFiducial) and data collection
(DataCollection) for Dynamic Extrinsics training.

PoseEstimator (base class)
  Support marker detection (at reduced scale) followed by pose estimation of the
  charuco board. This is the recommended way to use. Alternatively, pose estimation
  can be done directly (on the full scale input image) which takes longer to process.
  If pose estimation is successful, the client can call methods to extract chessboard
  corners, board pose (rvec and tvec), etc.  This is the way how pose estimation is 
  used in DataCollection.

PoseEstimatorWorld (sub-class)
  This class extends PoseEstimator to include Dynamic Extrinsics model, so that
  pose estimation results (rvec and tvec) are represented in the world coordinates.

PoseEstimation
├── CMakeLists.txt
├── PoseEstimator.cpp
├── PoseEstimator.h
├── README.txt
└── SixDOF.h

PoseEstimator.cpp/PoseEstimator.h
 Contains both PoseEstimator and PoseEstimatorWorld class definitions & some useful
 utility functions

SixDOF.h: structure for 6-DOF pose information
