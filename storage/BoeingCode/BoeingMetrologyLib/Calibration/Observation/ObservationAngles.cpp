#include "ObservationAngles.h"
#include <fstream>
#include "json/writer.h"
#include "json/reader.h"
#include "opencv2/highgui.hpp"

BoeingMetrology::Calibration::Observation::ObservationAngles::ObservationAngles()
{
    // 120 <= rx <= 240
    int numBins = 30;
    double min = 120;
    double max = 240;
    this->rx = Histogram(numBins, min, max);

    // -60 <= ry <= 60
    numBins = 30;
    min = -60;
    max = 60;
    this->ry = Histogram(numBins, min, max);

    // 0 <= rz <= 360
    numBins = 60;
    min = 0;
    max = 360;
    this->rz = Histogram(numBins, min, max);
}

void BoeingMetrology::Calibration::Observation::ObservationAngles::ModulateObservation(const double & rxsrc, const double & rysrc, const double & rzsrc, 
    double & rxdst, double & rydst, double & rzdst)
{
    rxdst = rxsrc;
    rydst = rysrc;
    rzdst = rzsrc;
    int numiter = 0;
    while (rxdst < 0.0)
    {
        rxdst += 360.0;
        if (++numiter > 10)
            throw std::runtime_error("ObservationAngles::ModulateObservation: rxsrc is uninitialized");
    }
    //while (rydst < 0.0)
    //    rydst += 360.0;
    numiter = 0;
    while (rzdst < 0.0)
    {
        rzdst += 360.0;
        if (++numiter > 10)
            throw std::runtime_error("ObservationAngles::ModulateObservation: rzsrc is uninitialized");
    }
}

void BoeingMetrology::Calibration::Observation::ObservationAngles::AddObservation(const double & rxnew, const double & rynew, const double & rznew)
{
    // Check validity
    try
    {
        this->rx.IsValid(rxnew);
    }
    catch (std::exception & e)
    {
        throw std::runtime_error(std::string("ObservationAngles::AddObservation RX ") + e.what());
    }
    try
    {
        this->ry.IsValid(rynew);
    }
    catch (std::exception & e)
    {
        throw std::runtime_error(std::string("ObservationAngles::AddObservation RY ") + e.what());
    }
    try
    {
        this->rz.IsValid(rznew);
    }
    catch (std::exception & e)
    {
        throw std::runtime_error(std::string("ObservationAngles::AddObservation RZ ") + e.what());
    }

    // Add the observation
    this->rx.AddValue(rxnew);
    this->ry.AddValue(rynew);
    this->rz.AddValue(rznew);
}

void BoeingMetrology::Calibration::Observation::ObservationAngles::JsonSerialize(Json::Value &jsonNode) const
{
    try
    {
        Json::Value jrx, jry, jrz;
        rx.JsonSerialize(jrx);
        jsonNode["rx"] = jrx;
        ry.JsonSerialize(jry);
        jsonNode["ry"] = jry;
        rz.JsonSerialize(jrz);
        jsonNode["rz"] = jrz;
    }
    catch (...)
    {
        throw std::runtime_error("ObservationAngles::JsonSerialize() failed");
    }
}

void BoeingMetrology::Calibration::Observation::ObservationAngles::JsonDeserialize(const Json::Value &jsonValue)
{
    try
    {
        this->rx.JsonDeserialize(jsonValue["rx"]);
        this->ry.JsonDeserialize(jsonValue["ry"]);
        this->rz.JsonDeserialize(jsonValue["rz"]);
    }
    catch (...)
    {
        throw std::runtime_error("ObservationAngles::JsonDeserialize() failed");
    }
}

void BoeingMetrology::Calibration::Observation::ObservationAngles::SerializeStream(std::ostream &strm) const
{
    // Camera info
    Json::Value root;
    JsonSerialize(root);
    Json::StyledStreamWriter json_writer;
    json_writer.write(strm, root);
}

void BoeingMetrology::Calibration::Observation::ObservationAngles::DeserializeStream(std::istream &strm)
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

void BoeingMetrology::Calibration::Observation::ObservationAngles::SerializeFile(std::string fileName) const
{
    //Write the serialization to the file
    std::ofstream strm(fileName);
    SerializeStream(strm);
    strm.close();
}

void BoeingMetrology::Calibration::Observation::ObservationAngles::WriteToCsv(const std::string & filename, const std::string & rstr)
{
    std::vector<double> binEdges;
    std::vector<size_t> binCounts;

    std::ofstream file;
    file.open(filename);
    if (rstr == "rx")
    {
        file << "RX Bin Edge (deg), RX Bin Count\n";
        this->rx.GetHistogram(binEdges, binCounts);
    }
    else if (rstr == "ry")
    {
        file << "RY Bin Edge (deg), RY Bin Count\n";
        this->ry.GetHistogram(binEdges, binCounts);
    }
    else if (rstr == "rz")
    {
        file << "RZ Bin Edge (deg), RZ Bin Count\n";
        this->rz.GetHistogram(binEdges, binCounts);
    }
    else
        throw std::runtime_error("ObservationAngles::WriteToCsv: Invalid rstr value");

    // Write the data
    for (size_t row = 0; row < binEdges.size(); row++)
    {
        file << binEdges[row] << ", " << binCounts[row] << std::endl;
    }

    // Close the file
    file.close();
}

void BoeingMetrology::Calibration::Observation::ObservationAngles::DeserializeFile(std::string fileName)
{
    //Read the content from the file and de-serialize into the class
    std::ifstream strm(fileName);
    DeserializeStream(strm);
    strm.close();
}

void BoeingMetrology::Calibration::Observation::ObservationAngles::WriteHistImage(const std::string & filename, const std::string & rstr /*= "rx"*/)
{
    cv::Mat img;
    std::pair<double, double> minMaxBin;
    size_t maxCount;
    std::ofstream file;
    file.open(filename);
    if (rstr == "rx")
    {
        this->rx.GetImage(cv::Scalar(0, 0, 255), img, minMaxBin, maxCount);
    }
    else if (rstr == "ry")
    {
        this->ry.GetImage(cv::Scalar(0, 255, 0), img, minMaxBin, maxCount);
    }
    else if (rstr == "rz")
    {
        this->rz.GetImage(cv::Scalar(255, 0, 0), img, minMaxBin, maxCount);
    }
    else
        throw std::runtime_error("ObservationAngles::WriteHistImage: Invalid rstr value");

    // Write the data
    cv::imwrite(filename, img);
}
