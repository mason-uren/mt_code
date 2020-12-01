#ifndef BOEINGMETROLOGYLIB_INSPECTIONFEATURES_H
#define BOEINGMETROLOGYLIB_INSPECTIONFEATURES_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "../Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    namespace Features
    {
        class BOEINGMETROLOGYLIB_API InspectionFeatures : public Boeing::Interface::Serializer
        {
        public:
            using HoleList = std::vector<cv::Vec4d>; // [x,y,z,radius]

            InspectionFeatures();

            virtual void JsonSerialize(Json::Value &jsonNode) const override;
            virtual void JsonDeserialize(const Json::Value &jsonNode) override;

            HoleList& getHoles() { return holes_; }
            const HoleList& getHoles() const { return holes_; }

            double getRadiusTolerance() const { return radiusTolerance_; }
            void setRadiusTolerance(double val) { radiusTolerance_ = val; }

            double getCenterTolerance() const { return centerTolerance_; }
            void setCenterTolerance(double val) { centerTolerance_ = val; }

        private:

            HoleList holes_;
            double radiusTolerance_;
            double centerTolerance_;
        };
    }
}

#endif
