#include "CollectionGenerator.h"

void CollectionGenerator::generatePositionMap() {
	this->calculatePoses();

	// Create increasing sequence of indexes...
	this->indexes = std::vector<int>(this->positions.pan.size() * this->positions.tilt.size());
	std::iota(this->indexes.begin(), this->indexes.end(), 0);

	
	// TODO - involves how points are returned by "getNextPoint/hasNextPoint"
	switch (this->getPatternEnum()) {
		case Presets::DataCollection::ScanningPattern::Enum::HORIZONTAL_RASTER:
			// Slots
			this->slots[PRIMARY] = this->collectionConfig.pan.slots;
			this->slots[SECONDARY] = this->collectionConfig.tilt.slots;

			// Primary Axis
			this->primaryAxis = Presets::PanTilt::Axis::Type::TILT;
			break;
		case Presets::DataCollection::ScanningPattern::Enum::VERTICAL_RASTER:
			// Slots
			this->slots[PRIMARY] = this->collectionConfig.tilt.slots; 
			this->slots[SECONDARY] = this->collectionConfig.pan.slots; 

			// Primary Axis
			this->primaryAxis = Presets::PanTilt::Axis::Type::PAN;
			break;
		case Presets::DataCollection::ScanningPattern::Enum::RANDOM:
			std::random_shuffle(this->indexes.begin(), this->indexes.end());

			// OLD
			// Slots
			//this->slots[PRIMARY] = this->collectionConfig.pan.slots;
			//this->slots[SECONDARY] = this->collectionConfig.tilt.slots;

			//// Generate iteration indexes
			//this->primaryAxisIndexes = std::vector<int>(this->collectionConfig.pan.slots);
			//this->secondaryAxisIndexes = std::vector<int>(this->collectionConfig.tilt.slots);
			//std::iota(this->primaryAxisIndexes.begin(), this->primaryAxisIndexes.end(), 0);
			//std::iota(this->secondaryAxisIndexes.begin(), this->secondaryAxisIndexes.end(), 0);

			//// Shuffle (indexes)
			//std::random_shuffle(this->primaryAxisIndexes.begin(), this->primaryAxisIndexes.end());
			//std::random_shuffle(this->secondaryAxisIndexes.begin(), this->secondaryAxisIndexes.end());

			break;
		default:
			this->eHandle->report(
				"Unknown scanning pattern <" + this->scanPattern.pattern + ">",
				Shared::Error::WARN
			);
			break;
	}
	
	// Determine starting position...
	switch (this->getOriginEnum()) {
		case Presets::DataCollection::Origin::Enum::TOP_LEFT: 
			break;
		case Presets::DataCollection::Origin::Enum::TOP_RIGHT:
			std::reverse(this->positions.pan.begin(), this->positions.pan.end());
			break;
		case Presets::DataCollection::Origin::Enum::BOT_LEFT:
			std::reverse(this->positions.tilt.begin(), this->positions.tilt.end());
			break;
		case Presets::DataCollection::Origin::Enum::BOT_RIGHT:
			std::reverse(this->positions.pan.begin(), this->positions.pan.end());
			std::reverse(this->positions.tilt.begin(), this->positions.tilt.end());
			break;
		default:
			this->eHandle->report(
				"Unknown collection origin <" + this->scanPattern.origin + ">",
				Shared::Error::Severity::WARN
			);
	}
}

bool CollectionGenerator::hasNextPTPose() {
	// NOTE: set by <getNextOrderedPose()> or <getRandomPose()>
	return !this->shouldExit;
}

std::vector<double> CollectionGenerator::getNextPTPose() {
	return (this->scanPattern.pattern == Presets::DataCollection::ScanningPattern::RANDOM) ?
		this->getRandomPose() : this->getNextOrderedPose();
}

std::vector<double> CollectionGenerator::getNextPTPose(std::vector<int> & gridPosition) {
	auto rawIdx{ this->secondaryIter - this->numberOfShifts };
	if (this->scanPattern.pattern == Presets::DataCollection::ScanningPattern::RANDOM) {
		auto panIdx{ this->indexes[this->indexPtr] / this->positions.tilt.size() };
		auto tiltIdx{ this->indexes[this->indexPtr] % this->positions.tilt.size() };
		gridPosition = std::vector<int>{(int) panIdx, (int) tiltIdx};
		//gridPosition = std::vector<int>{this->primaryAxisIndexes[this->primaryIter], this->secondaryAxisIndexes[this->secondaryIter]};
	}
	else {
		gridPosition = std::vector<int>{this->primaryIter, this->secondaryIter};
	}

	return this->getNextPTPose();
}

std::vector<double> CollectionGenerator::getRandomPose() {
	/* IDEA:
	 *		- increment both iterators sequentially 
	 *		- when reach end of container, wrap pan/tilt iters, then left shift tilt
	 *		- exit condition? (shifts > slots)
	 */
	// Pan/Tilt collection point
	auto panIdx{this->indexes[this->indexPtr] / this->positions.tilt.size()};
	auto tiltIdx{ this->indexes[this->indexPtr] % this->positions.tilt.size()};
	this->indexPtr++;


	//auto primaryIdx{this->primaryAxisIndexes[this->primaryIter++]};
	//auto seconaryIdx{this->secondaryAxisIndexes[this->secondaryIter++]};
	auto pose{ std::vector<double>{this->positions.pan[panIdx], this->positions.tilt[tiltIdx]} };

	//if (this->primaryIter >= this->positions.pan.size() || this->secondaryIter >= this->positions.tilt.size()) {
	//	this->primaryIter = 0;
	//	this->secondaryIter = 0;

	//	// Left shift tilt container
	//	std::rotate(this->secondaryAxisIndexes.begin(), this->secondaryAxisIndexes.begin() + 1, this->secondaryAxisIndexes.end());
	//	this->numberOfShifts++;
	//}

	// Exit condition
	//this->shouldExit = this->numberOfShifts >= this->positions.tilt.size();
	this->shouldExit = this->indexPtr >= this->indexes.size();

	return pose;
}

std::vector<double> CollectionGenerator::getNextOrderedPose() {
	auto isPanPrimaryAxis{ this->primaryAxis == Presets::PanTilt::Axis::Type::PAN };
	auto panIDX{ isPanPrimaryAxis ? this->primaryIter : this->secondaryIter };
	auto tiltIDX{ isPanPrimaryAxis ? this->secondaryIter : this->primaryIter };

	// Pan/Tilt collection point
	auto pose{ std::vector<double>{this->positions.pan[panIDX], this->positions.tilt[tiltIDX]} };

	this->secondaryIter++;
	if (this->secondaryIter >= this->slots[SECONDARY]) {
		this->primaryIter++;
		this->secondaryIter = 0;
	}

	// Exit condition (expect primary axis to trigger exit)
	this->shouldExit = !(this->primaryIter < this->positions.pan.size() && this->secondaryIter < this->positions.tilt.size());

	return pose;
}

int CollectionGenerator::numberOfPoses() {
	return static_cast<int>(this->positions.pan.size() * this->positions.tilt.size());
}

void CollectionGenerator::resetPositionIterators() {
	this->primaryIter = 0;
	this->secondaryIter = 0;
}

void CollectionGenerator::printPositionMap() {
	std::cout << this->positionMap() << std::endl;
}

std::string CollectionGenerator::toString() {
	return this->positionMap();
}

std::string CollectionGenerator::positionMap() {
	auto sstream{ std::stringstream{} };
	sstream << "P/T Map\n===================" << std::endl;
	if (this->scanPattern.pattern == Presets::DataCollection::ScanningPattern::RANDOM) {
		int idx{};
		int panIdx{};
		int tiltIdx{};
		bool exitCond{};

		sstream << "Random points acquired from: Left -> Right" << std::endl;

		for (const auto & idx : this->indexes) {
			panIdx = this->indexes[idx] / this->positions.tilt.size();
			tiltIdx = this->indexes[idx] % this->positions.tilt.size();
			sstream << "(" << this->positions.pan[panIdx] << ","
						   << this->positions.tilt[tiltIdx] << ")";

			//if (!(idx % this->positions.tilt.size()) && idx) {
			//	sstream << std::endl;
			//}
		}

		exitCond = idx >= this->indexes.size();

		// OLD
		//std::vector<double> tiltAxis{this->positions.tilt};
		//std::vector<int> tiltIndexes{ this->secondaryAxisIndexes };
		//int panIDX{};
		//int tiltIDX{};
		//bool exitCond{};
		//int numWraps{};

		//while (!exitCond) {
		//	sstream << "(" << this->positions.pan[this->primaryAxisIndexes[panIDX++]] << "," 
		//		    << this->positions.tilt[tiltIndexes[tiltIDX++]] << ")";

		//	// Wrap pointers...
		//	if (panIDX >= this->positions.pan.size() || tiltIDX >= this->positions.tilt.size()) {
		//		panIDX = 0;
		//		tiltIDX = 0;

		//		std::rotate(tiltIndexes.begin(), tiltIndexes.begin() + 1, tiltIndexes.end());
		//		numWraps++;

		//		// Line break
		//		sstream << std::endl;
		//	}

		//}
	}
	else {
		// Print normally (indifferent to scan pattern)
		for (auto & tilt : this->positions.tilt) {
			for (auto & pan : this->positions.pan) {
				sstream << "(" << pan << "," << tilt << ")";
			}
			sstream << std::endl;
		}
	}
	return sstream.str();
}

Presets::DataCollection::ScanningPattern::Enum CollectionGenerator::getPatternEnum() {
	if (this->scanPattern.pattern == Presets::DataCollection::ScanningPattern::HORIZONTAL_RASTER) {
		return Presets::DataCollection::ScanningPattern::Enum::HORIZONTAL_RASTER;
	}
	if (this->scanPattern.pattern == Presets::DataCollection::ScanningPattern::VERTICAL_RASTER) {
		return Presets::DataCollection::ScanningPattern::Enum::VERTICAL_RASTER;
	}
	if (this->scanPattern.pattern == Presets::DataCollection::ScanningPattern::RANDOM) {
		return Presets::DataCollection::ScanningPattern::Enum::RANDOM;
	}

	this->eHandle->report("Using default scanning patter: <" + Presets::DataCollection::ScanningPattern::HORIZONTAL_RASTER + ">");
	return Presets::DataCollection::ScanningPattern::Enum::HORIZONTAL_RASTER;
}

Presets::DataCollection::Origin::Enum CollectionGenerator::getOriginEnum() {
	if (this->scanPattern.origin == Presets::DataCollection::Origin::TOP_LEFT) {
		return Presets::DataCollection::Origin::Enum::TOP_LEFT;
	}
	if (this->scanPattern.origin == Presets::DataCollection::Origin::TOP_RIGHT) {
		return Presets::DataCollection::Origin::Enum::TOP_RIGHT;
	}
	if (this->scanPattern.origin == Presets::DataCollection::Origin::BOT_LEFT) {
		return Presets::DataCollection::Origin::Enum::BOT_LEFT;
	}
	if (this->scanPattern.origin == Presets::DataCollection::Origin::BOT_RIGHT) {
		return Presets::DataCollection::Origin::Enum::BOT_RIGHT;
	}

	this->eHandle->report("Using default board origin: TOP_LEFT");
	return Presets::DataCollection::Origin::Enum::TOP_LEFT;
}

void CollectionGenerator::calculatePoses() {
	this->positions.pan = std::vector<double>(this->collectionConfig.pan.slots);
	this->positions.tilt = std::vector<double>(this->collectionConfig.tilt.slots);

	int i{};
	for (auto & pose : this->positions.pan) {
		auto delta{ this->collectionConfig.pan.range[1] - this->collectionConfig.pan.range[0]};
		pose = PoseGenerator(this->collectionConfig.pan.range[0], delta, this->collectionConfig.pan.slots, i++);
	}

	int j{};
	for (auto & pose : this->positions.tilt) {
		auto delta{ this->collectionConfig.tilt.range[1] - this->collectionConfig.tilt.range[0] };
		pose = PoseGenerator(this->collectionConfig.tilt.range[0], delta, this->collectionConfig.tilt.slots, j++);
	}
}