#ifndef METROLOGY2020_DATACOLLECTION_H
#define METROLOGY2020_DATACOLLECTION_H

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <numeric>

// HRL
#include <Shared/DataCollectionConfig.h>

#include "ErrorHandler/ErrorHandler.h"

// pan_pos = [(start_pan + (i * (end_pan - start_pan)/Pan_Slots)) for i in range(Pan_Slots)]
// tilt_pos = [(start_tilt + (i * (end_tilt - start_tilt) / (Tilt_Slots / 1))) for i in range(Tilt_Slots)]
constexpr double PoseGenerator(const double & initPose, const double & delta, const int & slots, const int & iter) {
	return initPose + (iter * (delta / (slots > 1 ? slots - 1: slots)));
}

class CollectionGenerator {
public:
	CollectionGenerator(const DataCollection::CollectionConfig & collectionConfig, const DataCollection::ScanPattern & scanPattern, const double & seed = 0) :
		collectionConfig(collectionConfig),
		scanPattern(scanPattern),
		eHandle(ErrorHandler::getInstance()),
		shouldExit(false),
		slots(2, 0),
		primaryAxis(Presets::PanTilt::Axis::Type::PAN),
		primaryIter{},
		secondaryIter{},
		numberOfShifts{},
		primaryAxisIndexes{},
		secondaryAxisIndexes{}
	{
		std::srand(seed);
	}
	CollectionGenerator(const DataCollection::CollectionConfig & collectionConfig, const DataCollection::ScanPattern & scanPattern, const double & seed = 0, ErrorHandler * errorHandler = ErrorHandler::getInstance()) :
		collectionConfig(collectionConfig),
		scanPattern(scanPattern),
		eHandle(errorHandler),
		shouldExit(false),
		slots(2, 0),
		primaryAxis(Presets::PanTilt::Axis::Type::PAN),
		primaryIter{},
		secondaryIter{},
		numberOfShifts{},
		primaryAxisIndexes{},
		secondaryAxisIndexes{}
	{
		std::srand(seed);
	}
	CollectionGenerator(const std::vector<double> & panRange, const std::vector<double> & tiltRange, const int & slots, const double & seed = 0) :
		collectionConfig{
			DataCollection::Domain{
				slots,
				panRange
			},
			DataCollection::Domain{
				slots,
				tiltRange
			}
		},
		scanPattern{
			DataCollection::ScanPattern{
				Presets::DataCollection::ScanningPattern::HORIZONTAL_RASTER,
				Presets::DataCollection::Origin::TOP_LEFT
			}
		},
		eHandle(ErrorHandler::getInstance()),
		shouldExit(false),
		slots(2, 0),
		primaryAxis(Presets::PanTilt::Axis::Type::PAN),
		primaryIter{},
		secondaryIter{},
		numberOfShifts{},
		primaryAxisIndexes{},
		secondaryAxisIndexes{}
	{
		std::srand(seed);
	}

	// Functions
	void generatePositionMap();
	bool hasNextPTPose();
	std::vector<double> getNextPTPose();
	std::vector<double> getNextPTPose(std::vector<int> & gridPosition);
	int numberOfPoses();
	void resetPositionIterators();
	void printPositionMap();
	std::string toString();

	// Variables
	DataCollection::Positions positions{};

private:
	// Functions
	Presets::DataCollection::ScanningPattern::Enum getPatternEnum();
	Presets::DataCollection::Origin::Enum getOriginEnum();
	void calculatePoses();
	std::vector<double> getRandomPose();
	std::vector<double> getNextOrderedPose();
	std::string positionMap();

	// Variables
	std::vector<int> indexes{};
	int indexPtr{};
	double randomSeed{ 0 };

	// OLD

	// Variables
	enum Order {
		PRIMARY = 0,
		SECONDARY
	};

	DataCollection::CollectionConfig collectionConfig{};
	DataCollection::ScanPattern scanPattern{};
	ErrorHandler * eHandle;
	
	bool shouldExit{};
	std::vector<int> slots{};
	std::string primaryAxis{};
	int primaryIter{};
	int secondaryIter{};
	int numberOfShifts{};

	// IDEA: 
	//	- keep pan/tilt grid the same, but change up how indexing is returned
	std::vector<int> primaryAxisIndexes{};
	std::vector<int> secondaryAxisIndexes{};
};
#endif // !METROLOGY2020_DATACOLLECTION_H
