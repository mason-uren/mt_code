//
// Created by U'Ren, Mason R (VEN) on 4/14/20.
//

#ifndef METROLOGY2020_ALGORITHMS_PREPROCESS_H
#define METROLOGY2020_ALGORITHMS_PREPROCESS_H

//// Necessary for MSVS to include M_PI
//#define _USE_MATH_DEFINES
//#undef __STRICT_ANSI__

#include <cmath>
#include <limits>
#include <string>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Eigen
#include <Eigen/Core>

// HRL
//#include <Shared/DynamicExtrinsicConfig.h>
#include <Shared/Presets.h>

// Utilities
#include "Matrix/Matrix3d.h"

// Models
#include "Camera/CameraModel.h"
#include "Board/FiducialModel.h"

template <typename Primitive = double, typename Key = std::string>
class Preprocessor {
public:
    Preprocessor() = default;
    explicit Preprocessor(const FiducialModel & charucoBoard) :
        chessBoardCorners(charucoBoard.CHESSBOARD_CORNERS())
    {}
    ~Preprocessor() = default;

    void shareChessBoardCorners(const typename Matrix3d<Primitive, Key>::E_Matrix & corners) {
        chessBoardCorners = corners;
    }

    void splitPanTilt(Matrix3d<Primitive, Key> & metrics, const Key & type = Presets::DataSet::PanTiltAngles::RAD) {

        // Buffers for splitting Pan and Tilt
        auto bufferLen{metrics.isRowMajor() ? (unsigned long long) metrics[type].rows() : (unsigned long long) metrics[type].cols()};

        /*
         * Developers' Note:
         * Ideal initialization:
         *		Primitive pan[bufferLen];
         *		Primitive tilt[bufferLen];
         *
         * Not allowed by MSVS compiler
         */
        auto pan{ std::vector<Primitive>(bufferLen) };
        auto tilt{ std::vector<Primitive>(bufferLen) };

        // Eigen slicing does not apply to underlying data pointer
        // Can't use Eigen::matrix.data()
        int idx{};
        for (auto i = 0; i < bufferLen; i++) {
            pan[idx] = metrics[type](i, (int)Interface::PanTilt::Axis::PAN);
            tilt[idx] = metrics[type](i, (int)Interface::PanTilt::Axis::TILT);
            idx++;
        }

        metrics.createBuffer(Presets::DataSet::Types::PAN, {bufferLen, 1});
        metrics.createBuffer(Presets::DataSet::Types::TILT, {bufferLen, 1});

        metrics.mapMatrix(pan.data(), Presets::DataSet::Types::PAN);
        metrics.mapMatrix(tilt.data(), Presets::DataSet::Types::TILT);
    }

    void transformAnglesInPlace(Matrix3d<Primitive, Key> & metrics, const Key & toType) {
        auto fromType{toType == Presets::DataSet::PanTiltAngles::RAD ?
                      std::string{Presets::DataSet::PanTiltAngles::DEG} : std::string{Presets::DataSet::PanTiltAngles::RAD}};
        metrics[toType] = convertAngleMeasure(toType) * metrics[fromType];
    }

    void stripMatricesAtIdx(Matrix3d<Primitive, Key> & metrics, const Key & targetType, const double & epsilon) {
        auto tilts{metrics[targetType].col((int) Interface::PanTilt::Axis::TILT)};
        auto minValue{tilts.minCoeff()};
        auto _indexesToRemove{getMatIdxBelowThresh(tilts, minValue + epsilon, metrics.isRowMajor())};

        auto indexesToRemove{typename Matrix3d<int, Key>::E_Matrix{
                typename Matrix3d<int, Key>::E_Map(_indexesToRemove.data(), _indexesToRemove.size(), 1)
        }};

        for (auto & idx : indexesToRemove.reshaped()) {
            for (auto & pair : metrics.metrics) {
                metrics.removeMat(pair.first, idx);
            }
            // Account for removed matrix
            indexesToRemove -= Eigen::Matrix<int, Eigen::Dynamic, 1>::Ones(indexesToRemove.size());
        }
    }


    /**
     * @fn static void undistortImageCorners(Metrics<Primitive, Key> & metrics, const std::vector<std::shared_ptr<Model::Camera::DistortionHeader<>>> & distortionMap)
     * @brief Undistort the camera matrix corners.
     *
     * Developers' Note: Assumes that Eigen::Matrix is rowMajor
     *
     * @tparam Primitive underlying dataset element type
     * @tparam Key dataset accessor
     * @param metrics extrinsics datasets
     * @param distortionMap map of associated datasets
     */
    void undistortImageCorners(Matrix3d<Primitive, Key> & metrics, const std::unordered_map<std::string, CameraModel<Primitive>> & cameraModels) {
        for (auto & model : cameraModels) {
            auto cameraMatrix{model.second.eigenMatrix.cameraMatrix}; // .intrinsics};
            auto distortionCoeff{model.second.eigenMatrix.distortionCoeffs}; //.distortionCoeff};

            // Un-fold mapped buffer (restore original form)
            auto uvCornerShape{metrics.getDimensions(model.second.uvCornersKey)};
            typename Matrix3d<Primitive, Key>::E_Matrix e_inBuf{
                    typename Matrix3d<Primitive, Key>::E_Map(metrics[model.second.uvCornersKey].data(),
                                                             uvCornerShape[0] * uvCornerShape[1], uvCornerShape[2])
            };

            cv::Mat outBuf{};
            cv::Mat inBuf{};
            cv::eigen2cv(e_inBuf, inBuf);

            cv::Mat intrscBuf{};
            cv::eigen2cv(cameraMatrix, intrscBuf);

            cv::Mat distrBuf{};
            cv::eigen2cv(distortionCoeff, distrBuf);

            // Undistorted image
            cv::undistortPoints(inBuf, outBuf, intrscBuf, distrBuf);

            // Re-fold for storage
            typename Matrix3d<Primitive, Key>::E_Matrix e_outBuf{
                    typename Matrix3d<Primitive, Key>::E_Map((Primitive *) outBuf.data, uvCornerShape[0],
                                                             uvCornerShape[1] * uvCornerShape[2])
            };

            // Map back to buffer (disregard old data)
            metrics.mapMatrix(e_outBuf.data(), model.second.uvCornersKey);

            // // Create NAN padded matrix N x (741 x M)
            auto rows{chessBoardCorners.cols()};
            int cols{(int) uvCornerShape[2]};

            auto padding{typename Matrix3d<Primitive, Key>::E_Matrix(uvCornerShape[0], rows * cols)};
            auto _padding{cv::Mat((int) uvCornerShape[0], (int) rows * cols, CV_64F, NAN)};

            cv::cv2eigen(_padding, padding);

//            std::cout << padding << std::endl;

            // Remove bad values
            for (auto &index : metrics.indicesWithValuesAbove(model.second.cornerIDKey, 0)) {
                // Row : matrix idx
                // Col : (x, y) position idx
//                std::cout << "Padding:\n" << padding.block(index.first, index.second * 2, 0, 2) << std::endl;
//                std::cout << "UV:\n" << metrics[model.header.uvCornerKey].block(index.first, index.second * 2, 0, 2) << std::endl;
                padding.block(index.first, index.second * 2, 0, 2) = metrics[model.second.uvCornersKey].block(
                        index.first, index.second * 2, 0, 2);

            }

            metrics[model.second.uvCornersKey] = padding;
        }
    }

private:
    // Functions
    Primitive convertAngleMeasure(const Key & toType) {
        return toType == Presets::DataSet::PanTiltAngles::RAD ? (M_PI / 180) : (180 / M_PI);
    }

    template <typename ReturnType>
    ReturnType fromPTUCoordsSpace(const typename Matrix3d<Primitive, Key>::E_Matrix & mat3d, const bool rowMajor = false) {
        auto _mat3d{mat3d};
        _mat3d *= -1;
        for (auto i = 0; i < mat3d.rows(); i++) {
            rowMajor ? _mat3d(i, (int) Interface::PanTilt::Axis::PAN) += 90 : _mat3d((int)Interface::PanTilt::Axis::PAN, i) += 90;
        }
        return _mat3d;
    }

    std::vector<int> getMatIdxBelowThresh(const typename Matrix3d<Primitive, Key>::E_Matrix & mat3d,
                                                 const Primitive & threshold,
                                                 const bool isRowMajor = false) {
        auto indexesToRemove{std::vector<int>{}};
        auto bufferLen{isRowMajor ? mat3d.rows() : mat3d.cols()};
        for (auto i = 0; i < bufferLen; i++) {
            if (mat3d(i) < threshold) {
                indexesToRemove.push_back(i);
            }
        }

        return indexesToRemove;
    }

    // Variables
    Matrix3d<double>::E_Matrix chessBoardCorners;
};

#endif //METROLOGY2020_ALGORITHMS_PREPROCESS_H
