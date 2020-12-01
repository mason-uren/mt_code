#ifndef BOEINGMETROLOGYLIB_HISTOGRAM_H
#define BOEINGMETROLOGYLIB_HISTOGRAM_H

#include <vector>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include <cv.h>

namespace BoeingMetrology
{
    // A class to encapsulate distance units and conversion factors
    class BOEINGMETROLOGYLIB_API Histogram : public Boeing::Interface::Serializer
    {
    private:
        size_t numBins = 256;
        double min = 0.0;
        double max = 255.0;
        double binSize;

        // Raw values
        std::vector<double> values;

        // Binned values
        std::vector<size_t> hist;

        // Left-hand bin edges in increasing order
        std::vector<double> binEdges;

        // Clear all
        void ClearAll();

        // Compute bin size
        void ComputeBinSize();

        // Initialize bin edges
        void InitializeBinEdges();

        // Initialize the histogram
        void InitializeHist();

        void SerializeStream(std::ostream &strm) const;

        void DeserializeStream(std::istream &strm);

    public:

        // Default constructor sets numBins = 256, min = 0.0, max = 255.0
        Histogram()
        {
            *this = Histogram(256, 0.0, 255.0);
        }

        // Initialize values
        Histogram(const size_t & numBinsIn, const double & minIn, const double & maxIn);

        // Add a value
        void AddValue(const double & value);

        // Check validity of a value.  Throws exception if invalid.
        void IsValid(const double & value);

        // Get the histogram bins and counts
        void GetHistogram(std::vector<double> & binEdges, std::vector<size_t> & hist);

        // Get an image with the histogram to display.  Increasing column is increasing angle. 
        // Decreasing row is increasing bin count. 
        void GetImage(const cv::Scalar & color, cv::Mat & img, std::pair<double, double> & minMaxEdge, size_t & maxCount) const;

        // Get the raw values
        void GetRawValues(std::vector<double> & values);

        void JsonSerialize(Json::Value &jsonNode) const override;

        void JsonDeserialize(const Json::Value &jsonValue) override;

        virtual void SerializeFile(std::string fileName) const;

        virtual void DeserializeFile(std::string fileName);
    };
}

#endif
