#include "CalibrationDatabaseManager.h"
#include "Scanning/DynamicIntrinsics/MultiLensStateIntrinsicData.h"

namespace BoeingMetrology
{
    namespace Calibration
    {
        const std::string CalibrationDatabaseManager::mostRecentIntrinsicUpdateCollectionName = "IntrinsicData";
        const std::string CalibrationDatabaseManager::mostRecentIntrinsicVerifiedUpdateCollectionName = "IntrinsicDataVerified";
        const std::string CalibrationDatabaseManager::mostRecentIntrinsicAggregateCollectionName = "IntrinsicDataArchive";
        const std::string CalibrationDatabaseManager::mostRecentIntrinsicVerifiedAggregateCollectionName = "IntrinsicDataVerifiedArchive";

        const std::string CalibrationDatabaseManager::mostRecentDynamicIntrinsicUpdateCollectionName = "IntrinsicDataDynamic";
        const std::string CalibrationDatabaseManager::mostRecentDynamicIntrinsicVerifiedUpdateCollectionName = "IntrinsicDataDynamicVerified";
        const std::string CalibrationDatabaseManager::mostRecentDynamicIntrinsicAggregateCollectionName = "IntrinsicDataDynamicArchive";
        const std::string CalibrationDatabaseManager::mostRecentDynamicIntrinsicVerifiedAggregateCollectionName = "IntrinsicDataDynamicVerifiedArchive";

        const std::string CalibrationDatabaseManager::mostRecentExtrinsicUpdateCollectionName = "ExtrinsicData";
        const std::string CalibrationDatabaseManager::mostRecentExtrinsicVerifiedUpdateCollectionName = "ExtrinsicDataVerified";
        const std::string CalibrationDatabaseManager::mostRecentExtrinsicAggregateCollectionName = "ExtrinsicDataArchive";
        const std::string CalibrationDatabaseManager::mostRecentExtrinsicVerifiedAggregateCollectionName = "ExtrinsicDataVerifiedArchive";

		std::string CalibrationDatabaseManager::calibrationQualityAggregateCollectionName = "CalibrationQualityArchive";

        MultiCameraIntrinsicData CalibrationDatabaseManager::GetMultiCameraDynamicIntrinsicDataWithinDateRange(const Json::Value & src,
            const std::pair<std::string, std::string> & dateRange, const std::map<CAMERA_NAME, Scanning::Configuration::LensState> & lensStates, std::vector<std::string> & unfoundCameras)
        {
            // Chronologically sort dynamic intrinsics per sensor and select the set for each sensor that are most recent for that range
            auto dynamicIntrinsicsPerSensor = GetMultiCameraCalibrationDataWithinDateRange<Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData>(src, dateRange, unfoundCameras);

            // Initialize result
            MultiCameraIntrinsicData multiCameraIntrinsicData;

            // Loop through sensors
            for (const auto & sensor : dynamicIntrinsicsPerSensor)
            {
                // Interpolate result
                std::pair<Scanning::Configuration::LensState, IntrinsicData> interpResult;
                //sensor.second.GetIntrinsicDataLinearlyInterpolate(lensStates.at(sensor.first), interpResult);
                sensor.second.GetIntrinsicDataNearestNeighbor(lensStates.at(sensor.first), interpResult);
                multiCameraIntrinsicData.cameraData[sensor.first] = interpResult.second;

                std::cout << "CalibrationDatabaseManager dynamic intrinsics selection: " << sensor.first << " current focus = " << lensStates.at(sensor.first).GetFocusValue() << ", nearest neighbor = " << interpResult.first.GetFocusValue() << ", timestamp = " << sensor.second.GetTimestamp() << std::endl;
            }

            return multiCameraIntrinsicData;
        }        
    }
}
