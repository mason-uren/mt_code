#include "EstimatePoseCharucoBoard.h"

tuple<cv::Ptr<cv::aruco::CharucoBoard>,cv::Mat> createBoard(int charucoX, int charucoY, float squareLength, float markerLength, cv::Ptr<cv::aruco::Dictionary> dictionary)
{
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(charucoX, charucoY, squareLength, markerLength, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(1000, 1000), boardImage);
    return make_tuple(board, boardImage);
}

// Returns the error vectors and corresponding norms for all detected charuco corners
tuple<vector<cv::Point2f>,vector<double>> charuco_reprojection_error(cv::Ptr<cv::aruco::CharucoBoard> board, vector<cv::Point2f> chorners,
    vector<int> chids, cv::Vec3d rvec, cv::Vec3d tvec, cv::Mat intrinsics, cv::Mat dist)
{
    vector<cv::Point2f> imagePoints;
    cv::projectPoints(board->chessboardCorners, rvec, tvec, intrinsics, dist, imagePoints);
    // Only look at the corners which were actually detected
    vector<cv::Point2f> relevImgPoints;
    for (int i = 0; i < chids.size(); i++)
    {
        relevImgPoints.push_back(imagePoints[chids[i]]);
    }
    imagePoints = relevImgPoints;
    // At this point imagePoints should contain the projections in order corresponding to the detected corner ids
    // So, imagePoints[i] and chorners[i] should correspond to the same corner

    vector<cv::Point2f> errVec;
    vector<double> err;

    for (int i = 0; i < imagePoints.size(); i++)
    {
        cv::Point2f diff;
        diff.x = chorners[i].x - imagePoints[i].x;
        diff.y = chorners[i].y - imagePoints[i].y;
        errVec.push_back(diff);
        // Compute the norm
        err.push_back(sqrt(diff.x * diff.x + diff.y * diff.y));
    }
    return make_tuple(errVec, err);
}

tuple<cv::Mat,double> get_pose(vector<vector<cv::Point2f>> markerCorners, vector<int> markerIds, vector<cv::Point2f> chorners,
    vector<int> chids, cv::Ptr<cv::aruco::CharucoBoard> board, cv::Mat intrinsics, cv::Mat dist, bool debug_verbose)
{
    cv::Mat extrinsics_4x4;
    cv::Vec3d rvec, tvec;
    bool valid = cv::aruco::estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, rvec, tvec);
    if (!valid)
    {
        cout << "Unable to extract pose" << endl;
        return make_tuple(extrinsics_4x4, -1);
    }
    //cout << "Successfully extracted pose" << endl;

    // Convert rvec to 3x3 rotation matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    //cout << "R matrix" << endl;
    //cout << R << endl;

    double temp[4][4];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            temp[i][j] = R.at<double>(i,j);
        }
        temp[i][3] = tvec[i];
        temp[3][i] = 0;
    }
    temp[3][3] = 1;

    // Convert array to Mat
    cv::Mat converted(4, 4, CV_64F, temp);
    extrinsics_4x4 = converted;
    //cout << "Extrinsics Composed " << endl;
    //cout << extrinsics_4x4 << endl;

    // Compute the mean reprojection error
    auto tup = charuco_reprojection_error(board, chorners, chids, rvec, tvec, intrinsics, dist);

    vector<double> err = get<1>(tup);

    // Now, compute the mean error
    double mean_err = 0;
    for (int i = 0; i < err.size(); i++)
    {
        mean_err += err[i] / err.size();
    }

    if (debug_verbose)
    {
        cout << "Number of detected markers: " << markerIds.size() << endl;
        cout << "Number of detected Charuco corners: " << chorners.size() << endl;
        cout << "TVEC: " << tvec[0] << ' ' << tvec[1] << ' ' << tvec[2] << ' ' << endl;
        cout << "RVEC: " << rvec[0] << ' ' << rvec[1] << ' ' << rvec[2] << ' ' << endl;
        cout << "Mean RMS Error: " << mean_err << endl << endl << endl;
    }

    return make_tuple(extrinsics_4x4, mean_err);
}

tuple<cv::Mat,double,vector<int>,vector<cv::Point2f>> estimate_Pose_Charucoboard(cv::Mat image, cv::Ptr<cv::aruco::CharucoBoard> board,
    cv::Mat intrinsics, cv::Mat dist, cv::Ptr<cv::aruco::Dictionary> dictionary)
{
    //cout << "Entering pose estimation " << endl;

    // First, detect markers in the image
    vector<int> markerIds;
    vector<vector<cv::Point2f>> markerCorners;
    // Use default parameters for now
    cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds);

    if (markerIds.size() <= 5)
    {
        cout << "Insufficient markers detected, board pose estimation failed" << endl;
    }

    //cout << "Number of markers detected: " << markerIds.size() << endl;

    // Now, find the corresponding charuco corners assuming detected at least 5 markers
    // Should get back the charuco corners and corresponding ids
    vector<cv::Point2f> chorners;
    vector<int> chids;
    // Note: Can pass in the intrinsics and dist here to change the underlying estimation algorithm, should check
    // if improves performance
    int ret = cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, chorners, chids);

    // Now get and return the pose and corresponding mean reprojection (RMS) error
    auto tup = get_pose(markerCorners, markerIds, chorners, chids, board, intrinsics, dist);
    cv::Mat extrinsics_4x4 = get<0>(tup);
    double reprojection_error = get<1>(tup);

    return make_tuple(extrinsics_4x4, reprojection_error, chids, chorners);
}
