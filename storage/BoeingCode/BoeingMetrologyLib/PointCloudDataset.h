#ifndef BOEINGMETROLOGYLIB_POINTCLOUDDATASET_H
#define BOEINGMETROLOGYLIB_POINTCLOUDDATASET_H

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    /**
    * A 'patch' of point cloud data scanned or reconstructed from one or more cameras/sensors
     * optionally including UV information and metadata
     */
    class BOEINGMETROLOGYLIB_API PointCloudPatch // PointCloudXYZUV... ?
    {
    public:
        // xyz position of points
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        // map of uv coordinates for each point in the cloud with camera name as key 
        std::map<std::string, pcl::PointCloud<pcl::PointUV>::Ptr> uvMaps;

        // patch transform - patch to world, could be what the stitching modifies...
        cv::Mat transform;

        // source timestamp?
    };

    /**
     * A complex dataset of multiple point cloud patches with UV information from multiple
     * cameras and optionally associated 2d camera images.
     */
    class BOEINGMETROLOGYLIB_API PointCloudDataset // PointCloudGroup.... ? 
    {
    public:
        // map of patches that make up the entire point cloud using algorithm appropriate names for each key 
        // (eg. projector name, camera pair etc.)
        std::map<std::string, std::shared_ptr<PointCloudPatch>> patches;

        // map of camera images related to the patches (and the uvMaps map for each patch) 
        // using camera names as the key
        std::map<std::string, cv::Mat> images;

        // does the whole dataset need a transform for any reason?
    };
};


#endif
