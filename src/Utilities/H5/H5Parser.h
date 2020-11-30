//
// Created by U'Ren, Mason R (VEN) on 4/1/20.
//

#ifndef METROLOGY2020_ALGORITHMS_H5PARSER_H
#define METROLOGY2020_ALGORITHMS_H5PARSER_H

#include <vector>
#include <array>
#include <string>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <map>

// OpenCV
#include  <opencv2/core/core.hpp>

// HDF5
#include "H5Cpp.h"

// HRL
#include <Shared/Presets.h>
#include <Shared/DynamicExtrinsicConfig.h>

#include "Matrix/Matrix3d.h"
#include "ErrorHandler/ErrorHandler.h"

struct Mat3d {
	cv::Mat mat{};
	std::vector<int> dimensions{};
};


template <typename Primitive, typename Key = std::string>
class H5Parser {
public:
    explicit H5Parser() :
        keys{
            Presets::DataSet::Types::UV_CORNERS_IMPERX,
            Presets::DataSet::Types::UV_CORNERS_XIMEA,
            Presets::DataSet::Types::COMBINED_EXT,
            Presets::DataSet::Types::IMPERX_CHIDS,
            Presets::DataSet::Types::IMPERX_EXT,
            Presets::DataSet::Types::XIMEA_CHIDS,
            Presets::DataSet::Types::XIMEA_EXT,
            Presets::DataSet::PanTiltAngles::DEG,
        }
    {}
    ~H5Parser() = default;

    bool loadH5Files(std::vector<Matrix3d<Primitive, Key>> & dataset, const std::array<std::string, 2> & filePaths) {
        for (auto & path : filePaths) {
            Matrix3d<Primitive, Key> metrics{};
            if (loadH5File(&metrics, path)) {
                dataset.push_back(metrics);
            }
        }
        return !dataset.empty();
    }

    bool loadH5File(Matrix3d<Primitive, Key> * metrics, const std::string & path) {
        bool result{};

        try {
            H5::Exception::dontPrint();
            auto file{H5::H5File(path, H5F_ACC_RDONLY)};
            std::string dsKey{};

            // Iterate target keys
            for (auto & key : keys) {
                // Check if dataset is populated
                if (!file.exists(key)) {
                    continue;
                }
                auto dataSet{file.openDataSet(key)};

                // Read dataset into Metrics
                if (!(result = readDataSet(*metrics, dataSet, key))) {
                    ErrorHandler::getInstance()->report(
                            "Unable to read dataset ( " + dsKey + ")",
                            Shared::Error::Severity::WARN
                    );
                }
            }

        }
        catch (H5::FileIException & err) {
            ErrorHandler::getInstance()->report(err.getDetailMsg(), Shared::Error::Severity::WARN);
        }
        catch (H5::DataSetIException & err) {
            ErrorHandler::getInstance()->report(err.getDetailMsg(), Shared::Error::Severity::WARN);
        }

        return result;
    }

	bool writeH5File(const std::unordered_map<std::string, Mat3d> & dataMap, const std::string & path) {
		bool result{};

		try {
			H5::Exception::dontPrint();
			auto file{ H5::H5File(path, H5F_ACC_TRUNC) };

			// Iterate target keys
			for (auto & pair : dataMap) {

				ErrorHandler::getInstance()->report("Pair: " + pair.first);
				
				// Define space for array
				auto dims{ std::vector<unsigned long long>(pair.second.dimensions.begin(), pair.second.dimensions.end()) };
				H5::DataSpace dataSpace(dims.size(), dims.data());

				// Define type
				auto dataType = [&]() {
					switch (pair.second.mat.type()) {
						case CV_32S:
							return H5::PredType::STD_I32LE;
						case CV_32F:
							return H5::PredType::IEEE_F32LE;
						case CV_64F:
							return H5::PredType::IEEE_F64LE;
						default:
							
							ErrorHandler::getInstance()->report(
									pair.first + ": Unhandled OpenCV mat type: ( " + std::to_string(pair.second.mat.type()) + "); Default: ( IEEE_F64LE)",
									Shared::Error::Severity::WARN
							);
							return H5::PredType::IEEE_F64LE;
					}
				};

				dataType().setOrder(H5T_ORDER_LE);

				// Create dataset
				auto dataSet{ H5::DataSet(file.createDataSet(pair.first, dataType(), dataSpace)) };

				// Write to file
				dataSet.write(pair.second.mat.data, dataType());
			}

			file.close();
			result = true;

		}
		catch (H5::FileIException &) {
			H5::FileIException::printErrorStack();
		}
		catch (H5::DataSetIException &) {
			H5::DataSetIException::printErrorStack();
		}

		return result;
	}

    bool writeH5File(Matrix3d<Primitive, Key> & metrics, const std::string & path) {
		bool result{};

        try {
            H5::Exception::dontPrint();
            auto file{H5::H5File(path, H5F_ACC_TRUNC)};

            // Iterate target keys
            for (auto & key : keys) {
				if (!metrics.getDimensions(key).size()) {
					continue;
				}

                // Define space for array
				// Note: Msg -> Matrix has no dimensions. See -> Metrics.h 
                auto dataSpace{H5::DataSpace(metrics.getDimensions(key).size(), metrics.getDimensions(key).data())};
                // Define type
                auto dataType{H5::PredType::IEEE_F64LE};
                dataType.setOrder( H5T_ORDER_LE );

                // Create dataset
                auto dataSet{H5::DataSet(file.createDataSet(key, dataType, dataSpace))};

                // Write to file
                dataSet.write(metrics[key].data(), H5::PredType::IEEE_F64LE);
            }

			file.close();
			result = true;

        } catch (H5::FileIException &) {
            H5::FileIException::printErrorStack();
        } catch (H5::DataSetIException &) {
            H5::DataSetIException::printErrorStack();
        }

        return result;
    }

private:
    std::vector<std::string> keys{};

    bool readDataSet(Matrix3d<Primitive, Key> &metrics, const H5::DataSet &dataset, const std::string &dsType) {
        bool status{};
        auto dataSpace{dataset.getSpace()};
        auto rank{dataSpace.getSimpleExtentNdims()};

		/*
		 * Developers' Note:
		 * Ideal initialization:
		 *		hsize_t dims[rank];
		 *
		 * Not allowed by MSVS compiler
		 */
		std::vector<hsize_t> dims(rank);
        auto ndims{dataSpace.getSimpleExtentDims(dims.data(), nullptr)};

        // Create Data buffers
        if ((status = metrics.createBuffer(dsType, dims.data(), rank, metrics.isRowMajor()))) {
            // Read data from file and populate Eigen matrices/vectors
            auto buffer{metrics.getBuffer(dsType)};
            Primitive * _buffer;
            switch (buffer->type) {
                case DE::Set::Buffer::VECTOR: _buffer = (Primitive *) malloc(sizeof(Primitive) * buffer->dims[0]); break;
                case DE::Set::Buffer::MATRIX_2d: _buffer = (Primitive *) malloc(sizeof(Primitive) * buffer->dims[0] * buffer->dims[1]); break;
                case DE::Set::Buffer::MATRIX_3d: _buffer = (Primitive *) malloc(sizeof(Primitive) * buffer->dims[0] * buffer->dims[1] * buffer->dims[2]); break;
            }
//
//            Primitive * _buffer{
//                buffer->type == DE::Set::Buffer::Type::MATRIX ?
//                    (Primitive *) malloc(sizeof(Primitive) * buffer->dims[0] * buffer->dims[1] * buffer->dims[2]) :
//                    (Primitive *) malloc(sizeof(Primitive) * buffer->dims[0] * buffer->dims[1])
//            };
            dataset.read(_buffer, H5::PredType::IEEE_F64LE);
            metrics.mapMatrix(_buffer, dsType);
        }
        return status;
    }

};

#endif //METROLOGY2020_ALGORITHMS_H5PARSER_H
