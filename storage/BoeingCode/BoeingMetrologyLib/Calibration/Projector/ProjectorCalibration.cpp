#include "ProjectorCalibration.h"
#include <opencv2/imgproc.hpp>


void BoeingMetrology::Calibration::ProjectorCalibration::ComputeProjectorObservations(const Observation::CameraObservation& cameraObs, const cv::Mat& patternImage, 
    const cv::Mat& minMaxImage, Observation::CameraObservation& projectorObs, const unsigned& threshold /* = 0.3 */, const unsigned& WINDOW_SIZE /* = 30 */)
{
    // Validate some inputs
    if (patternImage.empty() || minMaxImage.empty())
        throw std::runtime_error("ERROR: ComputeProjectorObservations: Projector image is empty!");

    // Loop through camera image plane coordinates
    for (int cornerIdx = 0; cornerIdx < (int)cameraObs.observedPoints.size(); cornerIdx++)
    {
        // Camera image plane coordinate
        const cv::Point2f & p = cameraObs.observedPoints[cornerIdx];

        // Projector image plane coordinate
        cv::Point2f q(0.0f, 0.0f);

        // Find an homography around p
        std::vector<cv::Point2f> img_points, proj_points;
        if (p.x > WINDOW_SIZE && p.y > WINDOW_SIZE && p.x + WINDOW_SIZE < patternImage.cols && p.y + WINDOW_SIZE < patternImage.rows)
        {
            for (unsigned h = (unsigned)p.y - WINDOW_SIZE; h < (unsigned)p.y + WINDOW_SIZE; h++)
            {
                const cv::Vec2f * row = patternImage.ptr<cv::Vec2f>(h);
                const cv::Vec2b * min_max_row = minMaxImage.ptr<cv::Vec2b>(h);
                //cv::Vec2f * out_row = out_pattern_image.ptr<cv::Vec2f>(h);
                for (unsigned w = (unsigned)p.x - WINDOW_SIZE; w < (unsigned)p.x + WINDOW_SIZE; w++)
                {
                    const cv::Vec2f & pattern = row[w];
                    const cv::Vec2b & min_max = min_max_row[w];
                    //cv::Vec2f & out_pattern = out_row[w];
                    if (INVALID(pattern))
                    {
                        continue;
                    }
                    if ((min_max[1] - min_max[0]) < static_cast<int>(threshold))
                    {   //apply threshold and skip
                        continue;
                    }

                    img_points.push_back(cv::Point2f((float)w, (float)h));
                    proj_points.push_back(cv::Point2f(pattern));

                    //out_pattern = pattern;
                }
            }

            // Find local homography of small window around this observation.  This is a mapping from the 
            // camera image coordinate to the projector image coordinate in a small neighborhood.
            cv::Mat H = cv::findHomography(img_points, proj_points);
            //std::cout << " H:\n" << H << std::endl;
            cv::Point3d Q = cv::Point3d(cv::Mat(H*cv::Mat(cv::Point3d(p.x, p.y, 1.0))));
            q = cv::Point2f((float)(Q.x / Q.z), (float)(Q.y / Q.z));
        }
        else
        {
            return;
        }

        // Save the projector image coordinate
        projectorObs.AddObservation(cameraObs.markerIdentifier[cornerIdx], q, cameraObs.controlPoints[cornerIdx]);
    }
}
