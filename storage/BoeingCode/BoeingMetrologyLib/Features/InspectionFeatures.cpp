#include "InspectionFeatures.h"

namespace BoeingMetrology
{
    namespace Features
    {

        InspectionFeatures::InspectionFeatures() :
            radiusTolerance_(0),
            centerTolerance_(0)
        {
        }

        void InspectionFeatures::JsonSerialize(Json::Value& jsonNode) const
        {
            Json::Value holesJson(Json::arrayValue);
            for (auto& h : holes_)
            {
                Json::Value center;
                center.append(h[0]);
                center.append(h[1]);
                center.append(h[2]);
                Json::Value item;
                item["radius"] = h[3];
                item["center"] = center;
                holesJson.append(item);
            }

            jsonNode["hole_pattern"] = holesJson;
            jsonNode["radius_tolerance"] = radiusTolerance_;
            jsonNode["center_tolerance"] = centerTolerance_;
        }

        void InspectionFeatures::JsonDeserialize(const Json::Value& jsonNode)
        {
            holes_.clear();
            radiusTolerance_ = centerTolerance_ = 0;
                        
            for (auto& holeJson : jsonNode["hole_pattern"])
            {
                holes_.push_back(cv::Vec4d());
                auto& hole = holes_.back();
                auto posJson = holeJson["center"];
                hole[0] = posJson[0].asDouble();
                hole[1] = posJson[1].asDouble();
                hole[2] = posJson[2].asDouble();
                hole[3] = holeJson["radius"].asDouble();
            }

            radiusTolerance_ = jsonNode["radius_tolerance"].asDouble();
            centerTolerance_ = jsonNode["center_tolerance"].asDouble();
        }

    }
}
