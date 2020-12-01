// PoseEstimator.h : Header file for determining the 6DOF pose of a fiducial in a single frame of video
//

#pragma once

//#include <Windows.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/types.hpp>
#include <vector>
#include <fstream>
#include <iostream>
#include "Models/PanTilt/PanTiltModel.h"

using namespace std;

struct SixDOF
{
    double  roll, pitch, yaw;
    double  x, y, z;
    int num_charuco_found;      // number of ChArUco squares found in frame

    double  pan, tilt;          // pan and tilt angles of the PT unit
};

struct CADModel
{
    // Structure to store the 15-parameter CAD model and load it from a JSON file

    double yaw_ximea_tilt, pitch_ximea_tilt, roll_ximea_tilt;
    double x_ximea_tilt, y_ximea_tilt, z_ximea_tilt;
    double x_tilt_pan, y_tilt_pan, z_tilt_pan;
    double yaw_base_imperx, pitch_base_imperx, roll_base_imperx;
    double x_base_imperx, y_base_imperx, z_base_imperx;
    string modelfile, comments;

    inline bool loadModelFile(string filename)
    {
        // Since our original JSON had no depth, I just turned it into a regular text file with the format
        // parameter:value. This seemed easier than actually working with someone else's JSON parser.

        ifstream ifs;
        ifs.open(filename, ios::in);
        if(!ifs) {
            cout << "ERROR! Unable to open CAD model file: " << filename << endl;
            return false;
        }

        string sz, key, value;
        while(ifs.good())
        {
            // Values must always occur in pairs on a single line, separated by a colon. 
            // I'm not checking for this or enforcing it, so beware.
            getline(ifs, key, ':');
            getline(ifs, value, '\n');

            cout << "key: " << key << "\t\t value: " << value << endl; // test the capture with a print statement

            // Now do a switch statement on the keys to capture the values in the correct struct fields.
            // Also, remember to cast the numeric values into the appropriate type, leaving comment fields alone.
            if (key == "yaw_ximea_tilt")            yaw_ximea_tilt = stod(value);
            else if (key == "pitch_ximea_tilt")     pitch_ximea_tilt = stod(value);
            else if (key == "roll_ximea_tilt")      roll_ximea_tilt = stod(value);
            else if (key == "x_ximea_tilt")         x_ximea_tilt = stod(value);
            else if (key == "y_ximea_tilt")         y_ximea_tilt = stod(value);
            else if (key == "z_ximea_tilt")         z_ximea_tilt = stod(value);
            else if (key == "x_tilt_pan")           x_tilt_pan = stod(value);
            else if (key == "y_tilt_pan")           y_tilt_pan = stod(value);
            else if (key == "z_tilt_pan")           z_tilt_pan = stod(value);
            else if (key == "yaw_base_imperx")      yaw_base_imperx = stod(value);
            else if (key == "pitch_base_imperx")    pitch_base_imperx = stod(value);
            else if (key == "roll_base_imperx")     roll_base_imperx = stod(value);
            else if (key == "x_base_imperx")        x_base_imperx = stod(value);
            else if (key == "y_base_imperx")        y_base_imperx = stod(value);
            else if (key == "z_base_imperx")        z_base_imperx = stod(value);
            else if (key == "Comments")             comments = value;
            else {
                cout << "ERROR! Unrecognized key: " << key << endl;
                return false;
            }
        }
        cout << endl;  // add an extra space in the console output

        modelfile = filename;
        ifs.close();
        return true;
    }
};

//#######################################################################################
class PoseEstimator
{
public:
    PoseEstimator() : usePantiltModel(false) {}
    ~PoseEstimator() {}

public:
    bool            init(string modelJSON, string ximea_distCoeffs, string ximea_intrinsics, const double scaleFactor=1.0F, bool Debug=true);
    bool            calc(cv::Mat &frame, const double pan, const double tilt, SixDOF *result);
    bool            init2(const string & jsonModelPath, bool activateModel=false);
    bool            detect_aruco_markers(cv::Mat &img, cv::Point2f & markerCorner, bool drawMarkers=false);
    double          get_focal_length();

// Operations
private:
    bool            estimate_pose_charucoboard_ximea(const cv::Mat &img, cv::Mat *extrinsics_4x4, double *reprojection_error, int *charuco_sq_found);

    bool            get_pose(const vector<vector<cv::Point2f> > &markers, const vector<int> &ids, const vector<cv::Point2f> &chorners, 
                             const vector<int> &chids, cv::Mat *extrinsics_4x4, double *reprojection_error);

    bool            solvePNP_Charuco_Board(const vector<vector<cv::Point2f> > &markers, const vector<int> &ids, cv::Mat *rvec, cv::Mat *tvec);

    double          charuco_reprojection_error(const vector<vector<cv::Point2f> > &corners, const vector<int> &ids, const cv::Mat &rvec, const cv::Mat &tvec) const;

    bool            dynamic_extrinsics_correct_order(const double tilt_rad, const double pan_rad);

    void            doRotation(double rx, double ry, double rz, cv::Mat *result);
    void            doTranslation(double tx, double ty, double tz, cv::Mat *result);

    bool            decompose_extrinsics(const cv::Mat &extrinsics_4x4, cv::Mat *rmat, cv::Mat *tvec) const;
    bool            mat2euler(const cv::Mat &rmat, cv::Mat *rvec) const;

// Attributes
private:
    CADModel        cad;                         // the CAD model stored in 'modelJSON'
    Model::PanTilt<double, DataSet::Key>  pt_model;
    bool            usePantiltModel;

    const float     squareLength = 0.0127F;      // These values are for the 4"x6" fiducial that we use for 6DOF estimation
    const float     markerLength = 0.009F;       // located in "Charuco_Specific/CharucoBoards.py"
    const int       charucoX = 8;
    const int       charucoY = 12;

    bool            debug_verbose;
    const bool      subsampling = false;

    cv::Mat         intrinsics;
    cv::Mat         distCoeffs;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict;
    cv::Ptr<cv::aruco::CharucoBoard> board;

    // These matrices are different for each frame and are constantly changing. However, we will declare them here
    // instead of locally so that we aren't constantly reallocating their memory for every frame.
    cv::Mat         r_base_imperx, t_base_imperx;
    cv::Mat         r_pan_base;
    cv::Mat         t_tilt_pan, r_tilt_pan;
    cv::Mat         t_ximea_tilt, r_ximea_tilt;
    cv::Mat         tf;
    
    // temporary storage for convenience
    cv::Mat rvec_tmp, tvec_tmp;
};
