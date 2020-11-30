/**
 * @file ModelError.h
 * @brief A collection of loss functions.
 *
 * @author Mason U'Ren
 * @email mruren@hrl.com
 */

#ifndef METROLOGY2020_ALGORITHMS_MODELERROR_H
#define METROLOGY2020_ALGORITHMS_MODELERROR_H

#include <chrono>
#include <string>

// OpenCV
#include <opencv2/core/eigen.hpp>

// HRL
#include <Shared/DynamicExtrinsicConfig.h>
#include <Shared/Presets.h>

#include "Matrix/Matrix3d.h"
#include "Model/Transformations.h"
#include "Model/PanTiltModel.h"
#include "Board/FiducialModel.h"

template <typename Primitive = double, typename Key = std::string>
class ModelError {
private:
    static Matrix3d<double>::E_Matrix chessBoardCorners;

public:
    ModelError() = default;
    explicit ModelError(const FiducialModel & charucoBoard) {
        ModelError::chessBoardCorners = charucoBoard.CHESSBOARD_CORNERS();
    }
    virtual ~ModelError() = default;

    void setChessBoardCorners(const typename Matrix3d<Primitive, Key>::E_Matrix & corners) {
        ModelError::chessBoardCorners = corners;
    }

    /**
     * @fn static Metrics<Primitive, Key> getChessBoardCorners(const Key & key, std::vector<typename Metrics<Primitive, Key>::E_Matrix> & boardPositions)
     * @brief Given a board fiducial pose, compute the chessboard corners
     *
     * @tparam Primitive underlying dataset element type
     * @tparam Key dataset accessor type
     * @param[in] key dataset accessor
     * @param[in] boardPositions camera poses [N x 4 x 4]
     * @return A 3d matrix representation of the ChArUco board corners
     */
    static typename Matrix3d<Primitive, Key>::E_Matrix getChessBoardCorners(typename Matrix3d<Primitive, Key>::E_Matrix & boardPositions) {
        // Poses
        // (240 x 9)
        auto _idxPtr{sqrt(boardPositions.cols())};
        auto poses{typename Matrix3d<Primitive, Key>::E_Matrix(boardPositions.rows(), 9)}; // FIXME dangerous
        poses << boardPositions(Eigen::all, Eigen::seqN(0 * _idxPtr, ModelError::chessBoardCorners.rows())),
                boardPositions(Eigen::all, Eigen::seqN(1 * _idxPtr, ModelError::chessBoardCorners.rows())),
                boardPositions(Eigen::all, Eigen::seqN(2 * _idxPtr, ModelError::chessBoardCorners.rows()));

        // Reshape (240 x 3) x 3
        typename Matrix3d<Primitive, Key>::E_Matrix rotationMatrices {
                typename Matrix3d<Primitive, Key>::E_Map(
                        poses.data(), boardPositions.rows() * ModelError::chessBoardCorners.rows(), ModelError::chessBoardCorners.rows())
        };

        // (240 x 3)
        auto translations{typename Matrix3d<Primitive, Key>::E_Matrix(boardPositions.rows(), ModelError::chessBoardCorners.rows())};
        translations << boardPositions(Eigen::all, 3), // (_idxPtr * 1) - 1 // FIXME dangerous
                boardPositions(Eigen::all, 7), // (_idxPtr * 2) - 1 // FIXME dangerous
                boardPositions(Eigen::all, 11); // (_idxPtr * 3) - 1 // FIXME dangerous

        // Reshape (240 x 3) x 1
        typename Matrix3d<Primitive, Key>::E_Matrix translationMatrices{
                typename Matrix3d<Primitive, Key>::E_Map(translations.data(), boardPositions.rows() * ModelError::chessBoardCorners.rows(), 1)
        };

        typename Matrix3d<Primitive, Key>::E_Matrix result{
                rotationMatrices * ModelError::chessBoardCorners +
                translationMatrices * Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic>::Ones(1, ModelError::chessBoardCorners.cols())};

        // (240 x 3) x 741
        return typename Matrix3d<Primitive, Key>::E_Matrix{
                typename Matrix3d<Primitive, Key>::E_Map(result.data(), ModelError::chessBoardCorners.rows(), ModelError::chessBoardCorners.cols() * boardPositions.rows())
        };
    }

    /**
     * @fn static typename Metrics<Primitive, Key>::E_Matrix getXimeaCorners(Model::PanTilt<Primitive, Key> & ptuModel,
                                                                      const typename Metrics<Primitive, Key>::E_Matrix & rotationVec
                                                                      const typename Metrics<Primitive, Key>::E_Matrix & translationVec)
     * @brief From a fiducial pose in world space represented by #rotationVec and #translationVec, get its corners in 3d coordinates in ximea space.
     * @tparam Primitive underlying matrix element type
     * @tparam Key matrix accessor type
     * @param ptuModel[in,out] N-parameter model to be fit
     * @param dataset[in,out] reference to loaded datasets (extrinsics, pan/tilts, uvCorners, etc.)
     * @param rotationVec[in] [3 x 1]
     * @param translationVec[in] [3 x 1]
     * @return chessboard corners in ximea world space
     */
    static typename Matrix3d<Primitive, Key>::E_Matrix getXimeaCorners(Model::PanTilt<Primitive, Key> & ptuModel,
                                                                       Matrix3d<Primitive, Key> & dataset,
                                                                       const typename Matrix3d<Primitive, Key>::E_Matrix & rotationVec,
                                                                       const typename Matrix3d<Primitive, Key>::E_Matrix & translationVec) {
        // Generate identity matrix (1 x 16)
        auto matDims{(unsigned long long) sqrt(dataset[Presets::DataSet::Types::XIMEA_EXT].cols())};
        auto extrinsics{Matrix3d<Primitive, Key>::createIdentityMatrices({1, matDims, matDims}, true)()};
        ptuModel.applyRigid(extrinsics, rotationVec, translationVec);

        // Transform matrix to 240 x 16
        typename Matrix3d<Primitive, Key>::E_Matrix corners{
                Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic>::Ones(dataset[Presets::DataSet::Types::XIMEA_EXT].rows(), 1) * extrinsics
        };

        // Tranform into Ximea space
        auto ximeaExtrinsics{ptuModel.apply(corners, dataset[Presets::DataSet::Types::PAN], dataset[Presets::DataSet::Types::TILT])};

        return ModelError::getChessBoardCorners(ximeaExtrinsics);
    }

    /**
     * @fn static typename Metrics<Primitive, Key>::E_Matrix dynamicExplodedLoss(Model::PanTilt<Primitive, Key> & ptuModel,
                                                                          Metrics<Primitive, Key> & dataset,
                                                                          const typename Metrics<Primitive, Key>::E_Matrix & rotationVec,
                                                                          const typename Metrics<Primitive, Key>::E_Matrix & translationVec)
     * @brief Computes the average error between the corners in 3d measured by ximea and those represented by #rotationVec and #translationVec.
     * @tparam Primitive underlying matrix element type
     * @tparam Key matrix accessor type
     * @param ptuModel[in,out] N-parameter model to be fit
     * @param dataset[in,out] reference to loaded datasets (extrinsics, pan/tilts, uvCorners, etc.)
     * @param rotationVec[in] [3 x 1]
     * @param translationVec[in] [3 x 1]
     * @return average error between corners.
     */
    static Primitive dynamicExplodedLoss(Model::PanTilt<Primitive, Key> & ptuModel,
                                         Matrix3d<Primitive, Key> & dataset,
                                         const typename Matrix3d<Primitive, Key>::E_Matrix & rotationVec,
                                         const typename Matrix3d<Primitive, Key>::E_Matrix & translationVec) {
        // TODO - duplicate; consider refactor
        static const cv::Size SHAPE(cv::Size(ModelError::chessBoardCorners.rows(), ModelError::chessBoardCorners.cols()));

        auto actual{ModelError::getXimeaCorners(ptuModel, dataset, rotationVec, translationVec)};
        auto estimated{ModelError::getChessBoardCorners(dataset[Presets::DataSet::Types::XIMEA_EXT])};

        assert(actual.rows() == estimated.rows());
        assert(actual.cols() == estimated.cols());

        auto cornersErr{typename Matrix3d<Primitive, Key>::E_Matrix(1, actual.cols())};
        cornersErr = (estimated - actual).colwise().norm();

        typename Matrix3d<Primitive, Key>::E_Matrix result{
                typename Matrix3d<Primitive, Key>::E_Map(cornersErr.data(), cornersErr.cols() / SHAPE.height, SHAPE.height)
        };

        return result.mean();
    }

    /**
     * @fn static typename Metrics<Primitive, Key>::E_Matrix staticExplodedNorms(Model::PanTilt<Primitive, Key> & ptuModel,  Metrics<Primitive, Key> & dataSet)
     * @brief Computes the average error between the corners in 3d measured by the camera extrinsics
     *
     * @tparam Primitive underlying dataset element type
     * @tparam Key dataset accessor tyep
     * @param[in,out] ptuModel pan tilt model
     * @param[in, out] dataSet extrinsics]
     * @return
     */
    static typename Matrix3d<Primitive, Key>::E_Matrix staticExplodedNorms(Model::PanTilt<Primitive, Key> & ptuModel,
                                                                           Matrix3d<Primitive, Key> & dataSet) {
        // TODO - duplicate; consider refactor
        static const cv::Size SHAPE(cv::Size(ModelError::chessBoardCorners.rows(), ModelError::chessBoardCorners.cols()));

        auto roughXimeaExt{ptuModel.applyInverse(dataSet[Presets::DataSet::Types::XIMEA_EXT],
                                                 dataSet[Presets::DataSet::Types::PAN],
                                                 dataSet[Presets::DataSet::Types::TILT])};


        // Received matrix shape: 3 x M st. M is (741 x 240)
        auto ximeaCorners{ModelError::getChessBoardCorners(roughXimeaExt)};
        auto imperXCorners{ModelError::getChessBoardCorners(dataSet[Presets::DataSet::Types::IMPERX_EXT])};

        assert(ximeaCorners.rows() == imperXCorners.rows());

        // 1 x (741 x 240)
        auto cornersErr{typename Matrix3d<Primitive, Key>::E_Matrix(1, ximeaCorners.cols())};
        cornersErr = (ximeaCorners - imperXCorners).colwise().norm();

        // Reshape to 240 x 741
        typename Matrix3d<Primitive, Key>::E_Matrix result{
                typename Matrix3d<Primitive, Key>::E_Map(cornersErr.data(), roughXimeaExt.rows(), SHAPE.height)
        };

        return result.rowwise().mean();
    }

    static Primitive staticExplodedLoss(Model::PanTilt<Primitive, Key> & ptuModel, Matrix3d<Primitive, Key> & dataSet) {
        return ModelError::staticExplodedNorms(ptuModel, dataSet).mean();
    }
};

// Static variable initialization
template<> Matrix3d<double>::E_Matrix ModelError<double>::chessBoardCorners{};

//namespace ModelError {
//    /**
//     * @fn static Metrics<Primitive, Key> getChessBoardCorners(const Key & key, std::vector<typename Metrics<Primitive, Key>::E_Matrix> & boardPositions)
//     * @brief Given a board fiducial pose, compute the chessboard corners
//     *
//     * @tparam Primitive underlying dataset element type
//     * @tparam Key dataset accessor type
//     * @param[in] key dataset accessor
//     * @param[in] boardPositions camera poses [N x 4 x 4]
//     * @return A 3d matrix representation of the ChArUco board corners
//     */
//    template<typename Primitive = double, typename Key = std::string>
//    static typename Matrix3d<Primitive, Key>::E_Matrix getChessBoardCorners(typename Matrix3d<Primitive, Key>::E_Matrix & boardPositions) {
//        // Poses
//        // (240 x 9)
//        auto _idxPtr{sqrt(boardPositions.cols())};
//        auto poses{typename Matrix3d<Primitive, Key>::E_Matrix(boardPositions.rows(), 9)}; // FIXME dangerous
//        poses << boardPositions(Eigen::all, Eigen::seqN(0 * _idxPtr, Model::Charuco::CHESSBOARD_CORNERS.rows())),
//                 boardPositions(Eigen::all, Eigen::seqN(1 * _idxPtr, Model::Charuco::CHESSBOARD_CORNERS.rows())),
//                 boardPositions(Eigen::all, Eigen::seqN(2 * _idxPtr, Model::Charuco::CHESSBOARD_CORNERS.rows()));
//
//        // Reshape (240 x 3) x 3
//        typename Matrix3d<Primitive, Key>::E_Matrix rotationMatrices {
//                typename Matrix3d<Primitive, Key>::E_Map(
//                        poses.data(), boardPositions.rows() * Model::Charuco::CHESSBOARD_CORNERS.rows(), Model::Charuco::CHESSBOARD_CORNERS.rows())
//        };
//
//        // (240 x 3)
//        auto translations{typename Matrix3d<Primitive, Key>::E_Matrix(boardPositions.rows(), Model::Charuco::CHESSBOARD_CORNERS.rows())};
//        translations << boardPositions(Eigen::all, 3), // (_idxPtr * 1) - 1 // FIXME dangerous
//                        boardPositions(Eigen::all, 7), // (_idxPtr * 2) - 1 // FIXME dangerous
//                        boardPositions(Eigen::all, 11); // (_idxPtr * 3) - 1 // FIXME dangerous
//
//        // Reshape (240 x 3) x 1
//        typename Matrix3d<Primitive, Key>::E_Matrix translationMatrices{
//            typename Matrix3d<Primitive, Key>::E_Map(translations.data(), boardPositions.rows() * Model::Charuco::CHESSBOARD_CORNERS.rows(), 1)
//        };
//
//        typename Matrix3d<Primitive, Key>::E_Matrix result{
//            rotationMatrices * Model::Charuco::CHESSBOARD_CORNERS +
//            translationMatrices * Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic>::Ones(1, Model::Charuco::CHESSBOARD_CORNERS.cols())};
//
//        // (240 x 3) x 741
//        return typename Matrix3d<Primitive, Key>::E_Matrix{
//            typename Matrix3d<Primitive, Key>::E_Map(result.data(), Model::Charuco::CHESSBOARD_CORNERS.rows(), Model::Charuco::CHESSBOARD_CORNERS.cols() * boardPositions.rows())
//        };
//    }


//    /**
//     * @fn static typename Metrics<Primitive, Key>::E_Matrix getXimeaCorners(Model::PanTilt<Primitive, Key> & ptuModel,
//                                                                      const typename Metrics<Primitive, Key>::E_Matrix & rotationVec
//                                                                      const typename Metrics<Primitive, Key>::E_Matrix & translationVec)
//     * @brief From a fiducial pose in world space represented by #rotationVec and #translationVec, get its corners in 3d coordinates in ximea space.
//     * @tparam Primitive underlying matrix element type
//     * @tparam Key matrix accessor type
//     * @param ptuModel[in,out] N-parameter model to be fit
//     * @param dataset[in,out] reference to loaded datasets (extrinsics, pan/tilts, uvCorners, etc.)
//     * @param rotationVec[in] [3 x 1]
//     * @param translationVec[in] [3 x 1]
//     * @return chessboard corners in ximea world space
//     */
//    template <typename Primitive = double, typename Key = std::string>
//    static typename Matrix3d<Primitive, Key>::E_Matrix getXimeaCorners(Model::PanTilt<Primitive, Key> & ptuModel,
//                                                                       Matrix3d<Primitive, Key> & dataset,
//                                                                       const typename Matrix3d<Primitive, Key>::E_Matrix & rotationVec,
//                                                                       const typename Matrix3d<Primitive, Key>::E_Matrix & translationVec) {
//        // Generate identity matrix (1 x 16)
//        auto matDims{(unsigned long long) sqrt(dataset[Presets::DataSet::Types::XIMEA_EXT].cols())};
//        auto extrinsics{Matrix3d<Primitive, Key>::createIdentityMatrices({1, matDims, matDims}, true)()};
//        ptuModel.applyRigid(extrinsics, rotationVec, translationVec);
//
//        // Transform matrix to 240 x 16
//        typename Matrix3d<Primitive, Key>::E_Matrix corners{
//                Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic>::Ones(dataset[Presets::DataSet::Types::XIMEA_EXT].rows(), 1) * extrinsics
//        };
//
//        // Tranform into Ximea space
//        auto ximeaExtrinsics{ptuModel.apply(corners, dataset[Presets::DataSet::Types::PAN], dataset[Presets::DataSet::Types::TILT])};
//
//        return ModelError::getChessBoardCorners(ximeaExtrinsics);
//    }

//    /**
//     * @fn static typename Metrics<Primitive, Key>::E_Matrix dynamicExplodedLoss(Model::PanTilt<Primitive, Key> & ptuModel,
//                                                                          Metrics<Primitive, Key> & dataset,
//                                                                          const typename Metrics<Primitive, Key>::E_Matrix & rotationVec,
//                                                                          const typename Metrics<Primitive, Key>::E_Matrix & translationVec)
//     * @brief Computes the average error between the corners in 3d measured by ximea and those represented by #rotationVec and #translationVec.
//     * @tparam Primitive underlying matrix element type
//     * @tparam Key matrix accessor type
//     * @param ptuModel[in,out] N-parameter model to be fit
//     * @param dataset[in,out] reference to loaded datasets (extrinsics, pan/tilts, uvCorners, etc.)
//     * @param rotationVec[in] [3 x 1]
//     * @param translationVec[in] [3 x 1]
//     * @return average error between corners.
//     */
//    template <typename Primitive = double, typename Key = std::string>
//    static Primitive dynamicExplodedLoss(Model::PanTilt<Primitive, Key> & ptuModel,
//                                         Matrix3d<Primitive, Key> & dataset,
//                                         const typename Matrix3d<Primitive, Key>::E_Matrix & rotationVec,
//                                         const typename Matrix3d<Primitive, Key>::E_Matrix & translationVec) {
//        // TODO - duplicate; consider refactor
//        static const cv::Size SHAPE(cv::Size(Model::Charuco::CHESSBOARD_CORNERS.rows(), Model::Charuco::CHESSBOARD_CORNERS.cols()));
//
//        auto actual{ModelError::getXimeaCorners(ptuModel, dataset, rotationVec, translationVec)};
//        auto estimated{ModelError::getChessBoardCorners(dataset[Presets::DataSet::Types::XIMEA_EXT])};
//
//        assert(actual.rows() == estimated.rows());
//        assert(actual.cols() == estimated.cols());
//
//        auto cornersErr{typename Matrix3d<Primitive, Key>::E_Matrix(1, actual.cols())};
//        cornersErr = (estimated - actual).colwise().norm();
//
//        typename Matrix3d<Primitive, Key>::E_Matrix result{
//                typename Matrix3d<Primitive, Key>::E_Map(cornersErr.data(), cornersErr.cols() / SHAPE.height, SHAPE.height)
//        };
//
//        return result.mean();
//    }

//    /**
//     * @fn static typename Metrics<Primitive, Key>::E_Matrix staticExplodedNorms(Model::PanTilt<Primitive, Key> & ptuModel,  Metrics<Primitive, Key> & dataSet)
//     * @brief Computes the average error between the corners in 3d measured by the camera extrinsics
//     *
//     * @tparam Primitive underlying dataset element type
//     * @tparam Key dataset accessor tyep
//     * @param[in,out] ptuModel pan tilt model
//     * @param[in, out] dataSet extrinsics]
//     * @return
//     */
//    template<typename Primitive = double, typename Key = std::string>
//    static typename Matrix3d<Primitive, Key>::E_Matrix staticExplodedNorms(Model::PanTilt<Primitive, Key> & ptuModel,
//                                                                           Matrix3d<Primitive, Key> & dataSet) {
//        // TODO - duplicate; consider refactor
//        static const cv::Size SHAPE(cv::Size(Model::Charuco::CHESSBOARD_CORNERS.rows(), Model::Charuco::CHESSBOARD_CORNERS.cols()));
//
//        auto roughXimeaExt{ptuModel.applyInverse(dataSet[Presets::DataSet::Types::XIMEA_EXT],
//                                                 dataSet[Presets::DataSet::Types::PAN],
//                                                 dataSet[Presets::DataSet::Types::TILT])};
//
//
//        // Received matrix shape: 3 x M st. M is (741 x 240)
//        auto ximeaCorners{ModelError::getChessBoardCorners(roughXimeaExt)};
//        auto imperXCorners{ModelError::getChessBoardCorners(dataSet[Presets::DataSet::Types::IMPERX_EXT])};
//
//        assert(ximeaCorners.rows() == imperXCorners.rows());
//
//        // 1 x (741 x 240)
//        auto cornersErr{typename Matrix3d<Primitive, Key>::E_Matrix(1, ximeaCorners.cols())};
//        cornersErr = (ximeaCorners - imperXCorners).colwise().norm();
//
//        // Reshape to 240 x 741
//        typename Matrix3d<Primitive, Key>::E_Matrix result{
//            typename Matrix3d<Primitive, Key>::E_Map(cornersErr.data(), roughXimeaExt.rows(), SHAPE.height)
//        };
//
//        return result.rowwise().mean();
//    }

//    template <typename Primitive = double, typename Key = std::string>
//    static Primitive staticExplodedLoss(Model::PanTilt<Primitive, Key> & ptuModel, Matrix3d<Primitive, Key> & dataSet) {
//        return ModelError::staticExplodedNorms(ptuModel, dataSet).mean();
//    }
//}

#endif //METROLOGY2020_ALGORITHMS_MODELERROR_H
