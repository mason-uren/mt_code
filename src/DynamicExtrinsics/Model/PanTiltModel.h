//
// Created by U'Ren, Mason R (VEN) on 4/30/20.
//

#ifndef METROLOGY2020_ALGORITHMS_PANTILTMODEL_H
#define METROLOGY2020_ALGORITHMS_PANTILTMODEL_H

#include <string>
#include <utility>
#include <vector>
#include <memory>
#include <fstream>
#include <limits>
#include <iomanip>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <nlohmann/json.hpp>

#include <Shared/DynamicExtrinsicConfig.h>
#include <Shared/Presets.h>

// Models
//#include "PT/PTModelInterface.h"

// Utilities
#include "Matrix/Matrix3d.h"
#include "Transformations.h"

namespace Model{

	static const std::string MODEL_KEY{ Presets::Model::Key };

    template <typename Primitive = double, typename Key = std::string>
    class PanTilt: public Transformations { //public PTModelInterface,
    public:
		PanTilt() = default; // : PTModelInterface() {}
        ~PanTilt() = default;

		// Copy construction and assignment
		PanTilt(const PanTilt & obj) :
			model(obj.model),
			data(obj.data)
			//,
			//PTU_KEYS(obj.PTU_KEYS)
		{}

		PanTilt & operator=(const PanTilt & obj) {
			model = obj.model;
			data = obj.data;
			//PTU_KEYS = obj.PTU_KEYS;
			return *this;
		}

		// Move contstruction and assignment
		PanTilt(PanTilt && obj) :
			model(obj.model),
			data(obj.data)
			//,
			//PTU_KEYS(obj.PTU_KEYS)
		{}

		PanTilt & operator=(PanTilt && obj) {
			model = std::move(obj.model);
			data = std::move(obj.data);
			//PTU_KEYS = std::move(obj.PTU_KEYS);
			return *this;
		}

        // Read only access!
        typename Matrix3d<Primitive, Key>::E_Matrix operator[](const Key & key) {
            // data(modelKey) => E_Matrix
            // model[key] => int
            return data().row(model[key]);
        }

        // Read only access!
        typename Matrix3d<Primitive, Key>::E_Matrix operator[](const std::vector<std::string> & range) {
            auto isSingleton{range.size() < 2};
            auto min{std::min(model[range[0]],
                              isSingleton ? std::numeric_limits<int>::max() : model[range[range.size() - 1]])};
            auto max{std::max(model[range[0]],
                              isSingleton ? std::numeric_limits<int>::min() : model[range[range.size() - 1]])};
            auto _range{max - min + 1};

            assert(_range <= data().rows());

            return data().block(min, 0, _range, 1);
        }

        void loadModel(bool emptyModel = false) {
            using json = nlohmann::json;
            json config{};
            loadModel(config);
        }

        void loadModel(const std::string & file, const std::string & modelKey = MODEL_KEY) {
            using json = nlohmann::json;

            json jsonObj{};
            std::ifstream fileHandle{file};
            if (!fileHandle.is_open()) {
                std::cout << "Failed to open file at path: [ " << file << "]" << std::endl;
                return;
            }

            fileHandle >> jsonObj;

            if (jsonObj.contains(modelKey)) {
                jsonObj = jsonObj.at(modelKey);
            }

            loadModel(jsonObj);
        }

        void loadModel(nlohmann::json & jsonConfig, const std::string & modelKey = MODEL_KEY) {
			/*
			 * Developers' Note: 
			 * Ideal initialization: 
			 *		Primitive buffer[jsonObj.size()];
			 *
			 * Not allowed by MSVS compiler
			 */
			std::vector<Primitive> buffer(PTU_KEYS.size()); // Replacement initialization
            for (auto i = 0; i < PTU_KEYS.size(); i++) {
                try {
                    buffer[i] = jsonConfig.contains(PTU_KEYS[i]) ? jsonConfig.at(PTU_KEYS[i]).template get<Primitive>() : 0;

                    // Pan Scale
                    if (PTU_KEYS[i].find(Presets::Model::Types::PAN_SCALE) != std::string::npos) {
                        buffer[i] = 1;
                    }
                    // Tilt Scale
                    if (PTU_KEYS[i].find(Presets::Model::Types::TILT_SCALE) != std::string::npos) {
                        buffer[i] = 1;
                    }

                    model.insert({PTU_KEYS[i], i});
                } catch (...) {
                    std::cerr << "Failed to read PTU Model Json" << std::endl;
                }
            }

            // Load Eigen Vector
            /*
			 * Developers' Note:
			 * Ideal initialization:
			 *		std::vector<unsigned long long> shape{sizeof(buffer) / sizeof(buffer[0]), 1};
			 *
			 * Not allowed by MSVS compiler
			 */
            data.createBuffer(modelKey, {buffer.size(), 1}, data.isRowMajor());
            data.mapMatrix(buffer.data(), modelKey);
        }

        typename Matrix3d<Primitive, Key>::E_Matrix getModelRef() {
            return data();
        }

        void saveModel(const std::string & file, const std::string & modelKey = MODEL_KEY) {
            using json = nlohmann::json;

            std::ofstream fileHandle{file};

            if (model.empty()) {
                std::cout << "Current model is empty. Nothing to save." << std::endl;
                return;
            }

            std::unordered_map<std::string, std::unordered_map<std::string, Primitive>> map{};
            map.insert({modelKey, std::unordered_map<std::string, Primitive>()});
            for (auto i = 0; i < PTU_KEYS.size(); i++) {
                map[modelKey].insert({PTU_KEYS[i], data()(i)});
            }

            json jsonObj{map};
            fileHandle << std::setw(4) << jsonObj[0] << std::endl;
        }

        void updateModel(const std::vector<Key> & keys, const Eigen::Matrix<Primitive, Eigen::Dynamic, 1> & values, const std::string & modelKey = MODEL_KEY) {
            for (auto i = 0; i < keys.size(); i++) {
                // Eigen vector of 1
                data()(model[keys[i]]) = values(i);
            }
        }

        void displayModel(const Key & modelKey = MODEL_KEY) {
            std::cout << "PTU Model Shape: ( " << data().rows() << ", " << data().cols() << ")" << std::endl;
            for (auto & pair : model) {
                std::cout << pair.first << " " << (*this)[pair.first] << std::endl; // data(modelKey)(pair.second)
            }
        }

        /**
         * FIXME - expects 1 x 16
         * @param extrinsic
         * @param panAngles
         * @param tiltAngles
         * @return
         */
        typename Matrix3d<Primitive, Key>::E_Matrix apply(const typename Matrix3d<Primitive, Key>::E_Matrix & extrinsics,
                                                          const typename Matrix3d<Primitive, Key>::E_Matrix & panAngles,
                                                          const typename Matrix3d<Primitive, Key>::E_Matrix & tiltAngles) {

            assert(panAngles.rows() == tiltAngles.rows());
            assert(panAngles.rows() == extrinsics.rows());

            auto result{extrinsics};

            applyRigid(result, (*this)[{Presets::Model::Types::RX_PTU, Presets::Model::Types::RZ_PTU}],
                                  (*this)[{Presets::Model::Types::X_PTU, Presets::Model::Types::Z_PTU}]);
#ifdef DEBUG_EXAMPLE
            std::cout << "Rigid:\n" << extrinsic << std::endl;
#endif

            applyPans(result, panAngles * (*this)[Presets::Model::Types::PAN_SCALE]); // TODO - remove negation
#ifdef DEBUG_EXAMPLE
            std::cout << "Pans:\n" << extrinsic << std::endl;
#endif

            applyZRigid(result, (*this)[Presets::Model::Types::RZ_OFFSET], (*this)[Presets::Model::Types::Z_OFFSET]);
#ifdef DEBUG_EXAMPLE
            std::cout << "Z-Rigid:\n" << extrinsic << std::endl;
#endif

            applyTilts(result, tiltAngles * (*this)[Presets::Model::Types::TILT_SCALE]); // TODO - remove negation
#ifdef DEBUG_EXAMPLE
            std::cout << "Tilts:\n" << extrinsic << std::endl;
#endif

            applyRigid(result, (*this)[{Presets::Model::Types::RX_CAMERA, Presets::Model::Types::RZ_CAMERA}],
                                  (*this)[{Presets::Model::Types::X_CAMERA, Presets::Model::Types::Z_CAMERA}]);
#ifdef DEBUG_EXAMPLE
            std::cout << "Rigid:\n" << extrinsic << std::endl;
#endif

            return result;
        }

        /**
         * FIXME - expects 4x4
         * @param extrinsic
         * @param panAngles
         * @param tiltAngles
         * @return
         */
        void apply(cv::Mat & extrinsic,
                   const typename Matrix3d<Primitive, Key>::E_Matrix & panAngles,
                   const typename Matrix3d<Primitive, Key>::E_Matrix & tiltAngles) {
            typename Matrix3d<Primitive, Key>::E_Matrix e_mat(extrinsic.rows, extrinsic.cols);
            //
            Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic> e_mat_colmaj{
                    Eigen::Map<Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic>>(e_mat.data(), extrinsic.cols, extrinsic.rows)
            };
            e_mat_colmaj.transposeInPlace();

            // NOTE: OpenCV::cv2eigen - does not support row-major eigen matrices
            cv::cv2eigen(extrinsic, e_mat_colmaj);

            e_mat_colmaj.transposeInPlace();

            e_mat = typename Matrix3d<Primitive, Key>::E_Matrix{
                    typename Matrix3d<Primitive, Key>::E_Map(e_mat_colmaj.data(), extrinsic.rows, extrinsic.cols)
            };
            //

            // Transform 4 x 4 to 1 x 16
            typename Matrix3d<Primitive, Key>::E_Matrix reshaped{
                    typename Matrix3d<Primitive, Key>::E_Map(e_mat.data(), 1, e_mat.rows() * e_mat.cols())
            };
            auto result{apply(reshaped, panAngles, tiltAngles)};
            // Transoform 1 x 16 to 4 x 4
            typename Matrix3d<Primitive, Key>::E_Matrix _reshaped{
                typename Matrix3d<Primitive, Key>::E_Map(result.data(), extrinsic.rows, extrinsic.cols)
            };
            cv::eigen2cv(_reshaped, extrinsic);
        }

        /**
         * FIXME - expects 4x4
         * @param extrinsic
         * @param panAngle
         * @param tiltAngle
         */
        void apply(cv::Mat & extrinsic, const Primitive & panAngle, const Primitive & tiltAngle) {
            typename Matrix3d<Primitive, Key>::E_Matrix e_mat(extrinsic.rows, extrinsic.cols);

            //
            Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic> e_mat_colmaj{
                    Eigen::Map<Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic>>(e_mat.data(), extrinsic.cols, extrinsic.rows)
            };
            e_mat_colmaj.transposeInPlace();

            // NOTE: OpenCV::cv2eigen - does not support row-major eigen matrices
            cv::cv2eigen(extrinsic, e_mat_colmaj);

            e_mat_colmaj.transposeInPlace();

            e_mat = typename Matrix3d<Primitive, Key>::E_Matrix{
                    typename Matrix3d<Primitive, Key>::E_Map(e_mat_colmaj.data(), extrinsic.rows, extrinsic.cols)
            };
            //

            typename Matrix3d<Primitive, Key>::E_Matrix points(1, 2);
            points(0, 0) = panAngle;
            points(0, 1) = tiltAngle;

            // Transform 4 x 4 to 1 x 16
            typename Matrix3d<Primitive, Key>::E_Matrix reshaped{
                    typename Matrix3d<Primitive, Key>::E_Map(e_mat.data(), 1, e_mat.rows() * e_mat.cols())
            };

            auto result{apply(reshaped, points.col(0), points.col(1))};

            // Transoform 1 x 16 to 4 x 4
            typename Matrix3d<Primitive, Key>::E_Matrix _reshaped{
                    typename Matrix3d<Primitive, Key>::E_Map(result.data(), extrinsic.rows, extrinsic.cols)
            };
            cv::eigen2cv(_reshaped, extrinsic);
        }

        /**
         * FIXME - expects 4x4
         * @param extrinsic
         * @param panAngle
         * @param tiltAngle
         */
        void apply(typename Matrix3d<Primitive, Key>::E_Matrix & extrinsic, const Primitive & panAngle, const Primitive & tiltAngle) {
            typename Matrix3d<Primitive, Key>::E_Matrix points(1, 2);
            points(0, 0) = panAngle;
            points(0, 1) = tiltAngle;

            // Transform 4 x 4 to 1 x 16
            typename Matrix3d<Primitive, Key>::E_Matrix reshaped{
                    typename Matrix3d<Primitive, Key>::E_Map(extrinsic.data(), 1, extrinsic.rows() * extrinsic.cols())
            };
            auto result{apply(reshaped.data(), points.col(0), points.col(1))};
            // Transoform 1 x 16 to 4 x 4
            typename Matrix3d<Primitive, Key>::E_Matrix _reshaped{
                    typename Matrix3d<Primitive, Key>::E_Map(result.data(), extrinsic.rows, extrinsic.cols)
            };
        }

        /**
         * FIXME - expects 1 x 16
         * @param localExtrinsics
         * @param panAngles
         * @param tiltAngles
         * @return
         */
        typename Matrix3d<Primitive, Key>::E_Matrix applyInverse(//Key key,
                                             typename Matrix3d<Primitive, Key>::E_Matrix & localExtrinsics,
                                             const typename Matrix3d<Primitive, Key>::E_Matrix & panAngles,
                                             const typename Matrix3d<Primitive, Key>::E_Matrix & tiltAngles) {

            auto relativeExtrinsics{Matrix3d<Primitive, Key>::createIdentityMatrices({(unsigned long long) panAngles.rows(), 4, 4}, true)};
            relativeExtrinsics() = apply(relativeExtrinsics(), panAngles, tiltAngles);

            assert(relativeExtrinsics().rows() == localExtrinsics.rows());

            const auto ROWS = (unsigned long long) relativeExtrinsics().rows();
            const auto COLS = relativeExtrinsics().cols();

            auto worldExtrinsics{typename Matrix3d<Primitive, Key>::E_Matrix{localExtrinsics.rows(), localExtrinsics.cols()}};
            auto matDim{sqrt(COLS)};

            for (auto i = 0; i < relativeExtrinsics().rows(); i++) {
                typename Matrix3d<Primitive, Key>::E_Matrix _relExt{
                    typename Matrix3d<Primitive, Key>::E_Map(relativeExtrinsics().row(i).data(), matDim, matDim)
                };
                typename Matrix3d<Primitive, Key>::E_Matrix _localExt{
                    typename Matrix3d<Primitive, Key>::E_Map(localExtrinsics.row(i).data(), matDim, matDim)
                };
                typename Matrix3d<Primitive, Key>::E_Matrix _worldExt{
                    typename Matrix3d<Primitive, Key>::E_Map(worldExtrinsics.row(i).data(), matDim, matDim)
                };

                _worldExt = _relExt.inverse() * _localExt;

                worldExtrinsics.row(i) = typename Matrix3d<Primitive, Key>::E_Map(_worldExt.data(), 1, worldExtrinsics.cols());
            }

            return worldExtrinsics;
        }

        /**
         * FIXME - expects 4 x 4
         * @param extrinsic
         * @param panAngle
         * @param tiltAngle
         */
        void applyInverse(cv::Mat & extrinsic, const Primitive & panAngle, const Primitive & tiltAngle) {
			typename Matrix3d<Primitive, Key>::E_Matrix e_mat(extrinsic.rows, extrinsic.cols);
			//
			Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic> e_mat_colmaj{
				Eigen::Map<Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic>>(e_mat.data(), extrinsic.cols, extrinsic.rows)
			};
			e_mat_colmaj.transposeInPlace();
			
			// NOTE: OpenCV::cv2eigen - does not support row-major eigen matrices 
            cv::cv2eigen(extrinsic, e_mat_colmaj);

			e_mat_colmaj.transposeInPlace();

			e_mat = typename Matrix3d<Primitive, Key>::E_Matrix{
				typename Matrix3d<Primitive, Key>::E_Map(e_mat_colmaj.data(), extrinsic.rows, extrinsic.cols)
			};
			//

            typename Matrix3d<Primitive, Key>::E_Matrix points(1, 2);
            points(0, 0) = panAngle;
            points(0, 1) = tiltAngle;

            // Transform 4 x 4 to 1 x 16
            typename Matrix3d<Primitive, Key>::E_Matrix reshaped{
                    typename Matrix3d<Primitive, Key>::E_Map(e_mat.data(), 1, e_mat.rows() * e_mat.cols())
            };

            auto result{applyInverse(reshaped, points.col(0), points.col(1))};
            // Transoform 1 x 16 to 4 x 4
            typename Matrix3d<Primitive, Key>::E_Matrix _reshaped{
                    typename Matrix3d<Primitive, Key>::E_Map(result.data(), extrinsic.rows, extrinsic.cols)
            };
            cv::eigen2cv(_reshaped, extrinsic);
        }

        // Variables
        std::unordered_map<Key, int> model{};

    private:
        // Variables
        Matrix3d<Primitive, Key> data{};
        const std::vector<std::string> PTU_KEYS {
                Presets::Model::Types::X_CAMERA,
                Presets::Model::Types::Y_CAMERA,
                Presets::Model::Types::Z_CAMERA,
                Presets::Model::Types::RX_CAMERA,
                Presets::Model::Types::RY_CAMERA,
                Presets::Model::Types::RZ_CAMERA,
                Presets::Model::Types::Z_OFFSET,
                Presets::Model::Types::RZ_OFFSET,
                Presets::Model::Types::PAN_SCALE,
                Presets::Model::Types::TILT_SCALE,
                Presets::Model::Types::X_PTU,
                Presets::Model::Types::Y_PTU,
                Presets::Model::Types::Z_PTU,
                Presets::Model::Types::RX_PTU,
                Presets::Model::Types::RY_PTU,
                Presets::Model::Types::RZ_PTU
        };
    };


};

#endif //METROLOGY2020_ALGORITHMS_PANTILTMODEL_H
