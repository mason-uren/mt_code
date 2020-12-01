#include "Histogram.h"
#include <algorithm>
#include <fstream>
#include "json/writer.h"
#include "json/reader.h"
#include "opencv2/imgproc.hpp"

void BoeingMetrology::Histogram::ClearAll()
{
    values.clear();
    hist.clear();
    binEdges.clear();
}

void BoeingMetrology::Histogram::ComputeBinSize()
{
    // this->binSize = (this->max - this->min + 1) / (double)this->numBins;
    this->binSize = (this->max - this->min) / (double)this->numBins;
}

void BoeingMetrology::Histogram::InitializeBinEdges()
{
    this->binEdges.clear();
    double current = this->min;
    while (current < this->max)
    {
        binEdges.push_back(current);
        current += this->binSize;
    }
}

void BoeingMetrology::Histogram::InitializeHist()
{
    this->hist.clear();
    this->hist.resize(this->numBins, 0);
}

BoeingMetrology::Histogram::Histogram(const size_t & numBinsIn, const double & minIn, const double & maxIn)
{
    if (numBinsIn <= 0)
        throw std::runtime_error("Histogram Error: Number of bins should be a positive integer");

    if (minIn >= maxIn)
        throw std::runtime_error("Histogram Error: Max - Min must be positive");

    this->numBins = numBinsIn;
    this->min = minIn;
    this->max = maxIn;

    this->ComputeBinSize();

    this->InitializeBinEdges();

    this->InitializeHist();
}

void BoeingMetrology::Histogram::AddValue(const double & value)
{
    // Check that the value is within bounds
    try
    {
        this->IsValid(value);
    }
    catch (...)
    {
        throw;
    }

    try
    {
        // Increment histogram
        size_t bin = std::lower_bound(this->binEdges.begin(), this->binEdges.end() - 1, value) - this->binEdges.begin();
        hist[bin] += 1;
    }
    catch (...)
    {
        throw std::runtime_error("Histogram Error: Cannot add value");
    }

    // Add the raw value
    values.push_back(value);
}

void BoeingMetrology::Histogram::IsValid(const double & value)
{
    if (value < this->min)
        throw std::runtime_error("Histogram error: Cannot add value because it is less than the lowest bin edge");
    if (value > this->max)
        throw std::runtime_error("Histogram error: Cannot add value because it is greater than the highest bin edge");
}

void BoeingMetrology::Histogram::GetHistogram(std::vector<double> & binEdgesList, std::vector<size_t> & histList)
{
    binEdgesList = this->binEdges;
    histList = this->hist;
}

void BoeingMetrology::Histogram::GetImage(const cv::Scalar & color, cv::Mat & img, std::pair<double, double> & minMaxEdge, size_t & maxCount) const
{
    // Preallocate the output
    const auto cBinSpacing = 6;
    const auto cTickMarkerSize = 2;
    const auto cTextedTickMarkerSize = 5;
    const auto cHistogramRows = 60;
    const auto cFontMinimalSpacing = 5;

    auto bgColor = cv::Scalar(200, 200, 200);
    auto axisColor = cv::Scalar(0, 0, 0);
    std::vector<cv::Mat> markers;

    int fontRows = 10;
    for (size_t binidx = 1; binidx < this->binEdges.size(); binidx++)
    {
        // Draw a text vertically
        std::stringstream ss;
        ss << binEdges[binidx - 1];
        auto text = ss.str();
        int baseLine;
        const double fontScale = 1;
        cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_PLAIN, fontScale, 1, &baseLine);
        cv::Mat textImg = cv::Mat(size, CV_8UC3, bgColor);
        cv::putText(textImg, text, cv::Point(0, textImg.rows), cv::FONT_HERSHEY_PLAIN, fontScale, axisColor);
        cv::Mat vertialTextImg;
        cv::flip(textImg.t(), vertialTextImg, 1);

        markers.push_back(vertialTextImg);
        fontRows = std::max(fontRows, vertialTextImg.rows);
    }

    maxCount = *std::max_element(hist.begin(), hist.end());
    int imageRows = std::max(std::max(cTickMarkerSize, cTextedTickMarkerSize), cHistogramRows) + fontRows;
    img = cv::Mat(imageRows, (int)binEdges.size() * cBinSpacing, CV_8UC3, bgColor);

    int rightMostMarkerX = 0;

    // tickmarkers
    for (size_t binidx = 1; binidx < this->binEdges.size(); binidx++)
    {
        int x = (int)binidx * cBinSpacing;

        const cv::Mat& vertialTextImg = markers[binidx - 1];

        int fontX = x - vertialTextImg.cols / 2;
        int fontY = imageRows - fontRows;

        int tickMarkerSize = cTickMarkerSize;
        // Place tick marker text when not overlapping and having the required spacing
        if (fontX > rightMostMarkerX + cFontMinimalSpacing &&
            fontX + vertialTextImg.cols <= img.cols &&
            fontY + vertialTextImg.rows <= img.rows)
        {
            cv::Rect roi(fontX, fontY, vertialTextImg.cols, vertialTextImg.rows);
            cv::Mat roiImg(img, roi);
            vertialTextImg.copyTo(roiImg);

            tickMarkerSize = cTextedTickMarkerSize;
            rightMostMarkerX = fontX + vertialTextImg.cols;
        }


        // Put a tick marker on the axis
        cv::line(img, cv::Point(x, fontY), cv::Point(x, imageRows - fontRows - tickMarkerSize), axisColor, 1);
    }

    // x-axis
    cv::line(img, cv::Point(0, imageRows - fontRows), cv::Point(img.cols, imageRows - fontRows), axisColor, 1);

    // Loop through bin edges and connect adjacent points
    for (size_t binidx = 1; binidx < this->binEdges.size(); binidx++)
    {
        int prevX = ((int)binidx - 1) * cBinSpacing;
        int x = (int)binidx * cBinSpacing;

        cv::line(img, cv::Point(prevX, (int)(imageRows - fontRows - 1.0 * cHistogramRows * hist[binidx - 1] / maxCount)),
            cv::Point(x, (int)(imageRows - fontRows - 1.0 * cHistogramRows * hist[binidx] / maxCount)), color, 1);
    }

    minMaxEdge.first = *std::min_element(binEdges.begin(), binEdges.end());
    minMaxEdge.second = *std::max_element(binEdges.begin(), binEdges.end());
}

void BoeingMetrology::Histogram::GetRawValues(std::vector<double> & valuesStore)
{
    valuesStore = this->values;
}

void BoeingMetrology::Histogram::JsonSerialize(Json::Value &jsonNode) const
{
    try
    {
        jsonNode["numBins"] = static_cast<Json::UInt64>(this->numBins);
        jsonNode["min"] = this->min;
        jsonNode["max"] = this->max;
        jsonNode["binSize"] = this->binSize;
        for (const auto & value : values)
            jsonNode["values"].append(value);
        for (const auto & value : hist)
            jsonNode["hist"].append(static_cast<Json::UInt64>(value));
        for (const auto & value : binEdges)
            jsonNode["binEdges"].append(value);
    }
    catch (...)
    {
        throw std::runtime_error("Histogram::JsonSerialize() failed");
    }
}

void BoeingMetrology::Histogram::JsonDeserialize(const Json::Value &jsonValue)
{
    try
    {
        this->ClearAll();
        this->numBins = jsonValue["numBins"].asInt();
        this->min = jsonValue["min"].asDouble();
        this->max = jsonValue["max"].asDouble();
        this->binSize = jsonValue["binSize"].asDouble();
        for (const auto & value : jsonValue["values"])
            this->values.push_back(value.asDouble());
        for (const auto & value : jsonValue["hist"])
            this->hist.push_back(value.asInt());
        for (const auto & value : jsonValue["binEdges"])
            this->binEdges.push_back(value.asDouble());
    }
    catch (...)
    {
        throw std::runtime_error("Histogram::JsonDeserialize() failed");
    }
}

void BoeingMetrology::Histogram::SerializeStream(std::ostream &strm) const
{
    // Camera info
    Json::Value root;
    JsonSerialize(root);
    Json::StyledStreamWriter json_writer;
    json_writer.write(strm, root);
}

void BoeingMetrology::Histogram::DeserializeStream(std::istream &strm)
{
    Json::Reader jsonReader;
    Json::Value root;
    //Read through the JSON file and collect the points
    if (jsonReader.parse(strm, root))
    {
        JsonDeserialize(root);
    }
    else
    {
        throw std::runtime_error("ObservationAngles::DeserializeStream(): Failed to parse json.");
    }
}

void BoeingMetrology::Histogram::SerializeFile(std::string fileName) const
{
    //Write the serialization to the file
    std::ofstream strm(fileName);
    SerializeStream(strm);
    strm.close();
}

void BoeingMetrology::Histogram::DeserializeFile(std::string fileName)
{
    //Read the content from the file and de-serialize into the class
    std::ifstream strm(fileName);
    DeserializeStream(strm);
    strm.close();
}
