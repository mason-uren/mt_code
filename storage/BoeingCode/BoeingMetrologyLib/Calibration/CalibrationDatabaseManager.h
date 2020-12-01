#ifndef BOEINGMETROLOGYLIB_CALIBRATIONDATABASEMANAGER_H
#define BOEINGMETROLOGYLIB_CALIBRATIONDATABASEMANAGER_H

#include <string>
#include "Calibration/MultiCameraIntrinsicData.h"
#include "Calibration/MultiCameraExtrinsicData.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    namespace Calibration
    {
        class BOEINGMETROLOGYLIB_API CalibrationDatabaseManager
        {
        public:
            // Get a chronologically sorted list of documents per sensor
            template <typename T = IntrinsicData>
            static std::map<CAMERA_NAME, std::map<std::string, T>> GetSortedDocsPerSensor(const Json::Value & src)
            {
                std::map<CAMERA_NAME, std::map<std::string, T>> sortedDocsPerSensor;

                // Loop through docs
                for (const auto & doc : src)
                {
                    // Deserialize and dump into the map
                    T docDeserialized;
                    try
                    {
                        docDeserialized.JsonDeserialize(doc);
                    }
                    catch (...)
                    {
                        std::cout << "WARNING: GetSortedDocsPerSensor skipping document that failed to parse." << std::endl;
                        continue;
                    }
                    if (docDeserialized.GetTimestamp().empty())
                    {
                        std::cout << "ERROR: Ignoring JsonDeserialize entry for " << docDeserialized.GetCameraName() << " which has no valid timestamp." << std::endl;
                    }
                    else
                    {
                        sortedDocsPerSensor[docDeserialized.GetCameraName()][docDeserialized.GetTimestamp()] = docDeserialized;
                    }
                }

                return sortedDocsPerSensor;
            }

            // Various intrinsic collection names
            const static std::string mostRecentIntrinsicUpdateCollectionName;
            const static std::string mostRecentIntrinsicVerifiedUpdateCollectionName;
            const static std::string mostRecentIntrinsicAggregateCollectionName;
            const static std::string mostRecentIntrinsicVerifiedAggregateCollectionName;

            // Various dynamic intrinsic collection names
            const static std::string mostRecentDynamicIntrinsicUpdateCollectionName;
            const static std::string mostRecentDynamicIntrinsicVerifiedUpdateCollectionName;
            const static std::string mostRecentDynamicIntrinsicAggregateCollectionName;
            const static std::string mostRecentDynamicIntrinsicVerifiedAggregateCollectionName;

            // Various extrinsic collection names
            const static std::string mostRecentExtrinsicUpdateCollectionName;
            const static std::string mostRecentExtrinsicVerifiedUpdateCollectionName;
            const static std::string mostRecentExtrinsicAggregateCollectionName;
            const static std::string mostRecentExtrinsicVerifiedAggregateCollectionName;

			//the calibration quality assessment aggregate collection
			static std::string calibrationQualityAggregateCollectionName;

            // Get most recent intrinsics (or extrinsics) for each sensor that fall within date range
            template <typename T = IntrinsicData>
            static std::map<CAMERA_NAME, T> GetMultiCameraCalibrationDataWithinDateRange(const Json::Value & src, const std::pair<std::string, std::string> & dateRange, 
                std::vector<std::string> & unfoundCameras)
            {
                // Initialize output
                std::map<CAMERA_NAME, T> multiCameraFilteredData;

                // Get a chronologically sorted list of documents per sensor
                std::map<CAMERA_NAME, std::map<std::string, T>> sortedDocs = CalibrationDatabaseManager::GetSortedDocsPerSensor<T>(src);

                // Loop through each camera
                for (const auto & camera : sortedDocs)
                {
                    // Loop through docs for this camera by decreasing order (so we get the largest (newest) key first)
                    for (auto docIter = camera.second.rbegin(); docIter != camera.second.rend(); ++docIter)
                     {
                        if (dateRange.first.empty() || docIter->second.GetTimestamp() >= dateRange.first)
                        {
                            if (dateRange.second.empty() || docIter->second.GetTimestamp() <= dateRange.second)
                            {
                                // This document is the most recent within the limits
                                multiCameraFilteredData[camera.first] = docIter->second;
                                break;
                            }
                        }
                    }
                    if (multiCameraFilteredData.count(camera.first) == 0)
                    {
                        unfoundCameras.push_back(camera.first);
                    }
                }

                return multiCameraFilteredData;
            }

            // Get most recent dynamic intrinsics for each sensor that fall within date range and interpolate based on current lens state
            static MultiCameraIntrinsicData GetMultiCameraDynamicIntrinsicDataWithinDateRange(const Json::Value & src, const std::pair<std::string, std::string> & dateRange,
                const std::map<CAMERA_NAME, Scanning::Configuration::LensState> & lensStates, std::vector<std::string> & unfoundCameras);

        };
    }
}

#endif