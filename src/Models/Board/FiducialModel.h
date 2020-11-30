//
// Created by U'Ren, Mason R (VEN) on 4/28/20.
//

#ifndef METROLOGY2020_ALGORITHMS_CHARUCOBOARD_H
#define METROLOGY2020_ALGORITHMS_CHARUCOBOARD_H

#include <memory>
#include <string>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>

// HRL
#include <Shared/ModelsConfig.h>

// Utilities
#include "ThreadInterface/ConfigInterface.h"
#include "Matrix/Matrix3d.h"

#include "ConfigParser/ModelsJSONParser.h"


class FiducialModel: public ConfigInterface {
public:
	explicit FiducialModel(const std::string & caller = Presets::Logger::DEFAULT_ID) : ConfigInterface(caller) {}
    FiducialModel(const Model::Board & boardModel,
                  const cv::aruco::PREDEFINED_DICTIONARY_NAME & dictionary = cv::aruco::DICT_4X4_1000,
				  const std::string & caller = Presets::Logger::DEFAULT_ID) :
            ConfigInterface(caller),
            boardDict(cv::aruco::getPredefinedDictionary(dictionary)),
            board(cv::aruco::CharucoBoard::create(boardModel.squaresX,
                                                  boardModel.squaresY,
                                                  boardModel.squareLength,
                                                  boardModel.markerLength,
                                                  boardDict)),
            chessBoardCorners(cv::Mat{cv::Mat(board->chessboardCorners).reshape(1).t()})
    {
		eHandler->report("(" + caller + ") Fiducial Model: SquaresX(" + std::to_string(boardModel.squaresX) + ") "
														  "SquaresY(" + std::to_string(boardModel.squaresY) + ") "
														  "SquareLength(" + std::to_string(boardModel.squareLength) + ") "
														  "MarkerLength(" + std::to_string(boardModel.markerLength) + ")",
			Shared::Error::Severity::DEBUG, caller);
	}

	explicit FiducialModel(const cv::Ptr<cv::aruco::CharucoBoard> & charucoBoard, const std::string & caller = Presets::Logger::DEFAULT_ID) :
		ConfigInterface(caller),
		board(charucoBoard),
		boardDict(board->dictionary),
		chessBoardCorners(cv::Mat{ cv::Mat(board->chessboardCorners).reshape(1).t() })
	{}

    explicit FiducialModel(const std::shared_ptr<ModelsJSONParser> & config,
                           const std::string & key = std::string{},
                           const cv::aruco::PREDEFINED_DICTIONARY_NAME & dictionary = cv::aruco::DICT_4X4_1000, 
						   const std::string & caller = Presets::Logger::DEFAULT_ID) :
            FiducialModel(key.empty() ? config->chArUcoBoards.begin()->second : config->chArUcoBoards.at(key), dictionary)
    {
        ConfigInterface::loadConfig(config.get());
    }

    explicit FiducialModel(ModelsJSONParser * config,
                           const std::string & key = std::string{},
                           const cv::aruco::PREDEFINED_DICTIONARY_NAME & dictionary = cv::aruco::DICT_4X4_1000) :
            FiducialModel(key.empty() ? config->chArUcoBoards.begin()->second : config->chArUcoBoards.at(key), dictionary)
    {
        ConfigInterface::loadConfig(config);
    }

	~FiducialModel() override = default;

	void setup(const Model::Board & boardParams, const cv::aruco::PREDEFINED_DICTIONARY_NAME & dictionary = cv::aruco::DICT_4X4_1000);
	void setup(const cv::Ptr<cv::aruco::CharucoBoard> & charucoBoard);
	cv::Mat getChessBoardCorners();
	cv::Ptr<cv::aruco::CharucoBoard> & getBoard();
	cv::aruco::Dictionary & getDictionary();


    template <typename Primitive = double, typename Key = std::string>
    typename Matrix3d<Primitive, Key>::E_Matrix CHESSBOARD_CORNERS() const {
        static const std::string CALLER{ typeid(&FiducialModel::CHESSBOARD_CORNERS<Primitive, Key>).name() };
        auto corners{chessBoardCorners};
//        Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic> e_mat_colmaj(corners.rows, corners.cols);
//        // NOTE: OpenCV::cv2eigen - does not support row-major eigen matrices
//        cv::cv2eigen(corners, e_mat_colmaj);
//
//        e_mat_colmaj.transposeInPlace();
//
//        typename Matrix3d<Primitive, Key>::E_Matrix result{
//                typename Matrix3d<Primitive, Key>::E_Map(e_mat_colmaj.data(), corners.cols, corners.rows)
//        };

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eMat{ corners.rows, corners.cols };
        try {
            cv::cv2eigen(corners, eMat);
        }
        catch (const cv::Exception & except) {
			ErrorHandler::getInstance()->report("(" + CALLER + ") " + except.what(), Shared::Error::Severity::KILL_AND_EXIT);
        }
        return eMat;
//        return result;
    }

private:
    cv::Ptr<cv::aruco::Dictionary> boardDict{};
    cv::Ptr<cv::aruco::CharucoBoard> board{};
    cv::Mat chessBoardCorners{};
};

#endif //METROLOGY2020_ALGORITHMS_CHARUCOBOARD_H
