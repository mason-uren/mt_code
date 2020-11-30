#include "FiducialModel.h"

void FiducialModel::setup(const Model::Board & boardParams, const cv::aruco::PREDEFINED_DICTIONARY_NAME & dictionary) {
	this->boardDict = cv::aruco::getPredefinedDictionary(dictionary);
	this->board = cv::aruco::CharucoBoard::create(
		boardParams.squaresX,
		boardParams.squaresY,
		boardParams.squareLength,
		boardParams.markerLength,
		boardDict);
	this->chessBoardCorners = cv::Mat{ cv::Mat(this->board->chessboardCorners).reshape(1).t() };
}

void FiducialModel::setup(const cv::Ptr<cv::aruco::CharucoBoard> & charucoBoard) {
	this->boardDict = charucoBoard->dictionary;
	this->board = charucoBoard;
	this->chessBoardCorners = cv::Mat{ cv::Mat(this->board->chessboardCorners).reshape(1).t() };
}

cv::Mat FiducialModel::getChessBoardCorners() {
	return this->chessBoardCorners;
}

cv::Ptr<cv::aruco::CharucoBoard> & FiducialModel::getBoard() {
	return this->board;
}

cv::aruco::Dictionary & FiducialModel::getDictionary() {
	return *this->board->dictionary;
}