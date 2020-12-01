#pragma once
#include <string>
#include <cv.h>
#include "ExtrinsicData.h"
#include <map>
#include "Interface/ICameraPairDistances.h"
#include <list>
#include "BoeingMetrologyLib_API.h"
#include <set>

namespace BoeingMetrology
{
	namespace Calibration
	{
        class BOEINGMETROLOGYLIB_API MultiCameraExtrinsicData : public Boeing::Interface::Serializer, public Interface::ICameraPairDistances
		{
		public:
			// Container for ExtrinsicData indexed by camera name
			std::map<CAMERA_NAME, ExtrinsicData> cameraData;

			// Populate this object from the input json.
			// Throws exception on failure.
			void JsonDeserialize(const Json::Value &jsonNode) override;

			// Populate the Json node from this object's data members.
			// Throws exception on failure.
			void JsonSerialize(Json::Value &jsonNode) const override;

			// Retrieve the distances between sensor pairs
			std::map<CAMERA_NAME_PAIR, double> getCameraPairDistance() const override;

			class ExtrinsicMetricEnum
			{
			public:
				enum Type : int
				{
					LocationDistance = 0,
					LocationAngleDistance = 1, 
				};
				static std::list<std::string> * GetList()
				{
					std::list<std::string> *list = new std::list<std::string>();
					list->push_back("Location Distance");
					list->push_back("Location and Angle Distance");

					return list;
				}
				static Type GetType(const std::string &val)
				{
					Type retType;
					if ("Location Distance" == val)
						retType = ExtrinsicMetricEnum::LocationDistance;
					else if ("Location and Angle Distance" == val)
						retType = ExtrinsicMetricEnum::LocationAngleDistance;
                    return retType;
				}
			};

            // Get an ordered set of camera names
            std::set<CAMERA_NAME> GetCameraNames() const
            {
                std::set<CAMERA_NAME> names;
                for (const auto & name : this->cameraData)
                    names.insert(name.first);
                return names;
            }

			// Retrieve the distances between sensor pairs
			double getExtrinsicDataDistance(ExtrinsicMetricEnum::Type type, const MultiCameraExtrinsicData &obj, bool &mismatchedIndices) const;

			// Provide the magnitude of the position vector between a pair of cameras
			// Throws exception on failure.
			double ComputeMagnitudeOfDistanceBetweenCameraPair(const CAMERA_NAME & camName1, const CAMERA_NAME & camName2) const;

			// Provide the position vector between a pair of cameras
			// Throws exception on failure.
			cv::Point3d ComputePositionVectorBetweenCameraPair(const CAMERA_NAME & camName1, const CAMERA_NAME & camName2) const;

			// Provide the magnitude of the position vector between every pair of cameras
			// Throws exception on failure.
			void ComputeMagnitudeOfDistanceBetweenAllCameraPairs(std::map < CAMERA_NAME_PAIR, double> & distances) const;

			// Provide the position vector pointing from camera 1 to camera 2 between every pair of cameras
			// Throws exception on failure.
			void ComputePositionVectorBetweenAllCameraPairs(std::map < CAMERA_NAME_PAIR, cv::Point3d> & distances) const;

            // Get the common reference camera's name.  Throws exception if no common reference camera is found.
            void GetCommonReferenceCameraName(std::string & refCameraName) const;

            // Redefine the reference camera.  This will transform all extrinsics into the new reference camera's frame.  
            // Throws exception if there is no common reference camera in the current data or if the specified 
            // camera is not found.
            void RedefineReferenceCamera(const std::string & newRefCameraName);

            // Given a depth measurement for one sensor, compute the depth measurement for another sensor.
            // This function makes simplifying assumptions that the two sensor's z-axes (optical axes) intersect
            // at the observed depth value
            void ComputeDepthAlongCameraZ(const std::pair<CAMERA_NAME, double> & src, const CAMERA_NAME & dstCameraName, double & dst) const;
		};
	}
}
