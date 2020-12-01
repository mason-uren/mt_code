#include "CameraCalibrationUtilities.h"
#if defined(_MSC_VER) && _MSC_VER <= 1800
#include <QDirIterator>
#endif
#include <iostream>
#include <fstream>
#include "json/reader.h"
#include <time.h>
#include <iomanip>

// Function to get all pose directories and files
//************************************
// Method:    getPoseDirectoriesAndFiles
// FullName:  CSIRO::OpenCV::CollectiveCameraCalibrationImpl::getPoseDirectoriesAndFiles
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: const std::string & pathname
// Parameter: std::vector<std::string> & poseDirs
// Parameter: std::map<std::string, std::vector<std::pair<std::string, std::string>>> & jsonCalibFiles
// Parameter: std::map<std::string, std::vector<std::pair<std::string, std::string>>> & cameraFiles
//       by camera vect of pair of filename, pose dir
//************************************
#if defined(_MSC_VER) && _MSC_VER <= 1800
void BoeingMetrology::CameraCalibrationUtilities::getPoseDirectoriesAndFiles(const std::string &pathname,
    std::vector<std::string>& poseDirs,
    std::map<std::string, std::vector<std::tuple<std::string, std::string, std::string>>>& cameraFiles)
{
    QString filePathString = QString::fromStdString(pathname);
    QDir dirListing(filePathString);
    dirListing.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);

    QDirIterator it(filePathString, QDir::Dirs | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);

    // Loop through pose directories
    while (it.hasNext())
    {
        it.next();
        QString absPath = it.filePath();
        QString dirName = it.path();

        if (dirName.contains("accumulate") || dirName.contains("dotDetections") || dirName.contains("ObservationViz") || dirName.contains("BadArucoCornerFiles"))
            continue;

        //Obtain the list of images for processing
        QDir fileListing(absPath);

        fileListing.setFilter(QDir::Files | QDir::NoDotAndDotDot);
        QStringList filterList;
        filterList.append("*.jpg");
        filterList.append("*.png");
        filterList.append("*.hdr");
        fileListing.setNameFilters(filterList);
        QFileInfoList fileListSet = fileListing.entryInfoList();

        // Loop through image files in this pose directory
        //std::cout << "Dir     " << fileListing.dirName().toStdString() << "\n";
        //std::cout << "Dir     " << absPath.toStdString() << "\n";
        int fileCountAdded = 0;
        for (int fileNum = 0; fileNum < fileListSet.count(); ++fileNum)
        {
            if (fileListSet[fileNum].absoluteFilePath().contains("accumulate") || fileListSet[fileNum].absoluteFilePath().contains("dotDetections") || 
                fileListSet[fileNum].absoluteFilePath().contains("ObservationViz") || fileListSet[fileNum].absoluteFilePath().contains("markerCorners")
                || fileListSet[fileNum].absoluteFilePath().contains("BadArucoCornerFiles"))
                continue;

            std::string cameraName = fileListSet[fileNum].fileName().toStdString();
            cameraName = cameraName.substr(0, cameraName.find_last_of('.'));

            std::string charucoJsonFileName = fileListSet[fileNum].absoluteFilePath().toStdString();
            charucoJsonFileName = charucoJsonFileName.substr(0, charucoJsonFileName.find_last_of('.')) + ".json";
            size_t start_pos = charucoJsonFileName.find(cameraName + ".json");
            charucoJsonFileName.replace(start_pos, (cameraName + ".json").length(), "ChArUco" + cameraName + ".json");

            ++fileCountAdded;

            cameraFiles[cameraName].push_back(
                std::make_tuple(fileListSet[fileNum].absoluteFilePath().toStdString(),
                absPath.toStdString(), charucoJsonFileName));

            //std::cout << "File       " << cameraName << "    " << fileListSet[fileNum].absoluteFilePath().toStdString() << "\n";
        }

        //Only add a pose directory if the file count is greater than zero
        if (fileCountAdded > 0)
            poseDirs.push_back(fileListing.dirName().toStdString());
    }
}

void BoeingMetrology::CameraCalibrationUtilities::FileParts(const std::string & fullFileName, std::string & dir, std::string & filename)
{
    QFileInfo qfo(QString::fromStdString(fullFileName));
    dir = qfo.absolutePath().toStdString();
    filename = qfo.fileName().toStdString();
}

std::string BoeingMetrology::CameraCalibrationUtilities::ReplaceSubString(const std::string & src, const std::string & searchStr, const std::string & replaceStr)
{
    QString srcq = QString::fromStdString(src);
    srcq.replace(QString::fromStdString(searchStr), QString::fromStdString(replaceStr));
    return srcq.toStdString();
}

void BoeingMetrology::CameraCalibrationUtilities::CleanFileName(const std::string & src, std::string & dst)
{
    QString srcq = QString::fromStdString(src);
    srcq.replace("\\\\", "/");
    srcq.replace("\\", "/");
    dst = srcq.toStdString();
}
#else // Visual studio 2015 and newer
#ifdef _MSC_VER
#include <filesystem>
namespace fs = std::experimental::filesystem;
#elif defined(__APPLE__)
#  include "TargetConditionals.h"
#  ifdef TARGET_OS_MAC
#      include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#  endif
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif
// Function to get all pose directories and files
//************************************
// @param[in]: const std::string & pathname
// @param[out]: std::vector<std::string> & poseDirs
// @param[out]: std::map<std::string, std::vector<std::pair<std::string, std::string>>> & cameraFiles
//       by camera vect of pair of filename, pose dir
// @returns void.
//************************************
void BoeingMetrology::CameraCalibrationUtilities::getPoseDirectoriesAndFiles(const std::string &pathname,
    std::vector<std::string>& poseDirs,
    std::map<std::string, std::vector<std::tuple<std::string, std::string, std::string>>>& cameraFiles)
{
    fs::path path(pathname);
    fs::directory_iterator dirIter(path);

    for (auto dirPath : dirIter)
    {
        std::string cleanStrDirPath;
        CleanFileName(dirPath.path().string(), cleanStrDirPath);

        std::string dirName = cleanStrDirPath.substr(cleanStrDirPath.find_last_of("/\\") + 1);
        int fileCountAdded = 0;
        if (dirName.find("accumulate") != std::string::npos || dirName.find("dotDetections") != std::string::npos ||
            dirName.find("ObservationViz") != std::string::npos || dirName.find("BadArucoCornerFiles") != std::string::npos)
        {
            continue;
        }

        try
        {
            auto subDir = fs::directory_iterator(dirPath.path());
            for (auto fullFileName : subDir)
            {
                std::string cleanFullFileName;
                CleanFileName(fullFileName.path().string(), cleanFullFileName);

                auto filename = fullFileName.path().filename().string();
                if (filename.at(0) == '.')
                {
                    // Skip hidden files/folders
                    continue;
                }
                auto ext = fullFileName.path().filename().extension().string();
                if (ext != ".jpg" && ext != ".png" && ext != ".hdr")
                {
                    continue;
                }
                if (filename.find("accumulate") != std::string::npos || filename.find("dotDetections") != std::string::npos ||
                    filename.find("ObservationViz") != std::string::npos || filename.find("markerCorners") != std::string::npos ||
                    filename.find("BadArucoCornerFiles") != std::string::npos)
                {
                    continue;
                }

                auto cameraName = filename.substr(0, filename.find_last_of('.'));
                std::cout << " Camera Name  : " << cameraName << std::endl;

                std::string charucoJsonFileName = filename;
                charucoJsonFileName = charucoJsonFileName.substr(0, charucoJsonFileName.find_last_of('.')) + ".json";
                size_t start_pos = charucoJsonFileName.find(cameraName + ".json");
                charucoJsonFileName.replace(start_pos, (cameraName + ".json").length(), "ChArUco" + cameraName + ".json");
                ++fileCountAdded;

                cameraFiles[cameraName].push_back( std::make_tuple(cleanFullFileName, cleanStrDirPath, (cleanStrDirPath + "/" + charucoJsonFileName)) );
            }
        }
        catch (...)
        {
            // LH: Skip files that are not valid directories.
            continue;
        }

        if (fileCountAdded > 0)
        {
            poseDirs.push_back(cleanStrDirPath);
        }
    }
}

/************************************************************************
@brief divides full file path in to directory name and file name
@param[in] fullFileName : a std::string cointaining full name of the file.
@param[out] dir : a std::string cointaining directory name for the file.
@param[out] filename : a std::string cointaining file name.
@returns void
************************************************************************/
void BoeingMetrology::CameraCalibrationUtilities::FileParts(const std::string & fullFileName, std::string & dir, std::string & filename)
{
    fs::path filePath(filename);
    filename = filePath.filename().string();
    dir = fullFileName.substr(0, fullFileName.find_last_of("/\\"));
}


/************************************************************************
@brief Replaces a part of string with different string
@param[in] src : a std::string cointaining source string.
@param[in] searchStr : a std::string string to be replaced from source string source string.
@param[in] replaceStr : a std::string string to replace with.
@returns std::string : new std::string after replaceing 'searchStr' from 'src' with 'replaceStr'.
************************************************************************/
std::string BoeingMetrology::CameraCalibrationUtilities::ReplaceSubString(const std::string & src, const std::string & searchStr, const std::string & replaceStr)
{
    std::string tempStr = src;
    return tempStr.replace(tempStr.find(searchStr), searchStr.length(), replaceStr);
}


/************************************************************************
@brief Replaces '\\' with '/' for a file path provided as std::string
@param[in] src : a std::string cointaining file path.
@param[out] dst : a std::string cointaining file path with replaced /.
@returns void.
************************************************************************/
void BoeingMetrology::CameraCalibrationUtilities::CleanFileName(const std::string & src, std::string & dst)
{
    dst = src;
    if (dst.find("\\") != std::string::npos)
    {
        std::replace(dst.begin(), dst.end(), '\\', '/');
    }
}
#endif //_MSC_VER >=1800

void BoeingMetrology::CameraCalibrationUtilities::ComputeDistributionStats(const std::vector<double> &distribution, double &mean, double &stddev)
{
    mean = 0.0;
    stddev = 0.0;

    if (distribution.size() == 0)
        return;

    for (double val : distribution)
    {
        mean += val;
    }
    mean /= distribution.size();

    double varianceSum = 0.0;
    for (double val : distribution)
    {
        varianceSum += (val - mean)*(val - mean);
    }
    stddev = std::sqrt(varianceSum / (distribution.size() - 1));
}
