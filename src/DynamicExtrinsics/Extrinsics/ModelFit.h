/**
 * @file Extrinsics.h
 * @brief A collection of linear solvers.
 *
 * @author Mason U'Ren
 * @email mruren@hrl.com
 */

#ifndef METROLOGY2020_ALGORITHMS_EXTRINSICS_H
#define METROLOGY2020_ALGORITHMS_EXTRINSICS_H

#include <memory>

#include <Eigen/Core>

// Jacob: "Magic black box minimization algorithm courtesy of scipy"
// Scipy.minimize() : solver => (DEFAULT: BFGS)
// BFGS - Broyden-Fletcher-Goldfar-SHanno numerical optimization
#include <LBFGS.h>

// HRL
#include <Shared/DynamicExtrinsicConfig.h>
#include <Shared/Presets.h>

// Models
#include <Board/FiducialModel.h>

#include "Model/PanTiltModel.h"
#include "Model/Transformations.h"
#include "Matrix/Matrix3d.h"

#include "ModelError.h"


template <typename Primitive = double, typename Key = std::string>
class DynamicExtrinsics: public ModelError<Primitive> {
public:
    explicit DynamicExtrinsics(const FiducialModel & charucoBoard,
                               const DE::Optimization::LFBGS & config = DE::Optimization::LFBGS{}) :
        ModelError<Primitive>(charucoBoard),
        finiteDiffEpsilon(1e-8)

//        charucoBoard(charucoBoard)
    {
        algParams.epsilon = config.epsilon ? config.epsilon : algParams.epsilon;
        algParams.delta = config.delta ? config.delta : algParams.delta;
        algParams.max_iterations = config.maxIterations ? config.maxIterations : algParams.max_iterations;
//        algParams.max_submin = config.maxSubMin ? config.maxSubMin : algParams.max_submin;
        algParams.max_linesearch = config.maxLineSearch ? config.maxLineSearch : algParams.max_linesearch;
        algParams.min_step = config.minStep ? config.minStep : algParams.min_step;
        algParams.max_step = config.maxStep ? config.maxStep : algParams.max_step;

        // Solver
        solver = std::unique_ptr<LBFGSpp::LBFGSSolver<Primitive>>(new LBFGSpp::LBFGSSolver<Primitive>(algParams));
    }
    ~DynamicExtrinsics() override = default;

    // Copy/Assignment Constructors
    DynamicExtrinsics(const DynamicExtrinsics<Primitive, Key> & obj) :
        algParams(obj.algParams),
        solver(std::unique_ptr<LBFGSpp::LBFGSSolver<Primitive>>(new LBFGSpp::LBFGSSolver<Primitive>(algParams)))
    {}
    DynamicExtrinsics & operator=(const DynamicExtrinsics<Primitive, Key> & obj) {
        algParams = obj.algParams;
        solver = std::unique_ptr<LBFGSpp::LBFGSSolver<Primitive>>(new LBFGSpp::LBFGSSolver<Primitive>(algParams));
    }

    // Move/Assignment Constuctors
    DynamicExtrinsics(DynamicExtrinsics<Primitive, Key> && obj)  noexcept :
        algParams(std::move(obj.algParams)),
        solver(std::move(obj.solver))
    {}
    DynamicExtrinsics & operator=(DynamicExtrinsics<Primitive, Key> && obj)  noexcept {
        algParams = std::move(obj.algParams);
        solver = std::move (obj.solver);
    }

    void fitDynamicParams(Model::PanTilt<Primitive, Key> & ptuModel,
                                 std::vector<Matrix3d<Primitive, Key>> & dataSets,
                                 const std::vector<Key> & paramsToFit,
                                 std::function<Primitive(Model::PanTilt<Primitive, Key> &,
                                                         Matrix3d<Primitive, Key> &,
                                                         const typename Matrix3d<Primitive, Key>::E_Matrix &,
                                                         const typename Matrix3d<Primitive, Key>::E_Matrix &)> lossFuncCallback) {
        if (paramsToFit.empty()) {
            std::cout << "No parameters to fit against model" << std::endl;
            return;
        }

        /**
         * Lambda Functions
         * - Objective
         * - Wrapper
         */
        auto Objective = [&](const Eigen::Matrix<Primitive, Eigen::Dynamic, 1> & x) -> Primitive {
            // Update model
            ptuModel.updateModel(paramsToFit, x.block(0, 0, paramsToFit.size(), 1));

            Eigen::Matrix<Primitive, Eigen::Dynamic, 1> losses(dataSets.size());
            for (auto i = 0; i < dataSets.size(); i++) {
                losses[i] = lossFuncCallback(
                        ptuModel,
                        dataSets[i],
                        x.block(paramsToFit.size() + 6 * i, 0, 3, 1),
                        x.block(paramsToFit.size() + 6 * i + 3, 0, 3, 1));
            }

            return losses.mean();
        };

        auto Wrapper = [&](Eigen::Matrix<Primitive, Eigen::Dynamic, 1> & x,
                           Eigen::Matrix<Primitive, Eigen::Dynamic, 1> & grad) {
            grad = finiteDifferenceApproximation(x, Objective);
            auto avgLoss{Objective(x)};
            std::cout << "\rLoss: " << avgLoss << std::flush;
            return avgLoss;
        };

        // Setup
        // 2 Datasets * (1 rvec + 1 tvec) s.t. rvec and tvec size is 3 = 12 + 6 paramaters to fit = 18 parameters
        Eigen::Matrix<Primitive, Eigen::Dynamic, 1> parameters{Eigen::Matrix<Primitive, Eigen::Dynamic, 1>::Zero(18)};
        for (auto j = 0; j < dataSets.size(); j++) {
            // (3 x 240)
            auto rotationVectors{typename Matrix3d<Primitive, Key>::E_Matrix(3, dataSets[j][Presets::DataSet::Types::XIMEA_EXT].rows())};
            // (240 x 3)
            auto translationVectors{typename Matrix3d<Primitive, Key>::E_Matrix(dataSets[j][Presets::DataSet::Types::XIMEA_EXT].rows(), 3)};

            auto extrinsics{ptuModel.applyInverse(dataSets[j][Presets::DataSet::Types::XIMEA_EXT],
                                                  dataSets[j][Presets::DataSet::Types::PAN],
                                                  dataSets[j][Presets::DataSet::Types::TILT])};

            for (auto i = 0; i < dataSets[j][Presets::DataSet::Types::XIMEA_EXT].rows(); i++) {
                typename Matrix3d<Primitive, Key>::E_Matrix mat(extrinsics.row(i));
                rotationVectors.col(i) = ptuModel.rotationMatToVec(mat);
            }

            translationVectors << extrinsics(Eigen::all, 3), // FIXME dangerous
                    extrinsics(Eigen::all, 7), // FIXME dangerous
                    extrinsics(Eigen::all, 11 ); // FIXME dangerous

            parameters.block((j * 6) + 6, 0, 3, 1) = rotationVectors.rowwise().mean();
            parameters.block((j * 6) + 9, 0, 3, 1) = translationVectors.colwise().mean().transpose();
        }

        // FIXME - dangerous!! assumes "paramsToFit" references first 6 positions in vector
        parameters.block(0, 0, 6, 1) = ptuModel[paramsToFit];
//        std::cout << parameters << std::endl;

        // Initial guess
        Eigen::Matrix<Primitive, Eigen::Dynamic, 1> x{parameters};

        std::cout << "*** Dynamic Fit ***" << std::endl;
        std::cout << "Params to fit: " << std::endl;
        for (auto & param : paramsToFit) {
            std::cout << param << " : " << ptuModel[param] << std::endl;
        }

        Primitive fx;

        // L-BFGS algorithm
        int iterations = solver->minimize(Wrapper, x, fx);
        displayResults(iterations, fx, x);
        // Update model
        ptuModel.updateModel(paramsToFit, x);
    }

    void fitStaticParams(Model::PanTilt<Primitive, Key> & ptuModel,
                                std::vector<Matrix3d<Primitive, Key>> & dataSets,
                                const std::vector<Key> & paramsToFit,
                                std::function<Primitive(Model::PanTilt<Primitive, Key> &, Matrix3d<Primitive, Key> &)> lossFuncCallback) {

        if (paramsToFit.empty()) {
            std::cout << "No parameters to fit against model" << std::endl;
            return;
        }

        /*
         * Lamba Functions
         * - Objective
         * - Wrapper
         */

        /**
         * @fn Objective
         */
        auto Objective = [&](const Eigen::Matrix<Primitive, Eigen::Dynamic, 1> & x) -> Primitive {
            // Update model
            ptuModel.updateModel(paramsToFit, x);

            Eigen::Matrix<Primitive, Eigen::Dynamic, 1> losses(dataSets.size());
            for (auto i = 0; i < dataSets.size(); i++) {
                losses[i] = lossFuncCallback(ptuModel, dataSets[i]);
            }

            return losses.mean();
        };

        /**
         * @fn Wrapper
         */
        auto Wrapper = [&](Eigen::Matrix<Primitive, Eigen::Dynamic, 1> & x,
                           Eigen::Matrix<Primitive, Eigen::Dynamic, 1> & grad) {
            grad = finiteDifferenceApproximation(x, Objective);
            auto avgLoss{Objective(x)};
            std::cout << "\rLoss: " << avgLoss << std::flush;
            return avgLoss;
        };

        // Initial guess
        Eigen::Matrix<Primitive, Eigen::Dynamic, 1> x{ptuModel[paramsToFit]};

        std::cout << "*** Static Fit ***" << std::endl;
        std::cout << "Params to fit:" << std::endl;
        for (auto & param : paramsToFit) {
            std::cout << param << " : " << ptuModel[param] << std::endl;
        }

        Primitive fx;

        // L-BFGS algorithm
        int iterations = solver->minimize(Wrapper, x, fx);
        displayResults(iterations, fx, x);
        // Update model
        ptuModel.updateModel(paramsToFit, x);
    }

private:
    // Functions

    /**
     * @fn finiteDifferenceApproximation
     * @brief Finite difference approximation of the gradient of a scalar function.
     */
    auto finiteDifferenceApproximation(const Eigen::Matrix<Primitive, Eigen::Dynamic, 1> & x,
                                       std::function<Primitive(const Eigen::Matrix<Primitive, Eigen::Dynamic, 1> &)> ObjectiveFunc)
                                       -> decltype(Eigen::Matrix<Primitive, Eigen::Dynamic, 1>{}) {
//        const Primitive EPSILON{1e-08};
        Eigen::Matrix<Primitive, Eigen::Dynamic, 1> grad{Eigen::Matrix<Primitive, Eigen::Dynamic, 1>::Zero(x.rows())};
        Eigen::Matrix<Primitive, Eigen::Dynamic, 1> d{Eigen::Matrix<Primitive, Eigen::Dynamic, 1>::Zero(x.rows())};

        // Base Case
        auto f0{ObjectiveFunc(x)};

        for (auto i = 0; i < x.rows(); i++) {
            // Assuming Eigen vector notation
            d(i) = finiteDiffEpsilon;
            auto df{
                    (ObjectiveFunc(x + d) - f0) / d[i]
            };
            // TODO - maybe check if value is scalar
            grad(i) = df;
            // Unset EPSILON
            d(i) = 0;
        }

        return grad;
    }

    void displayResults(const int & iterations, const Primitive & fx, const Eigen::Matrix<Primitive, Eigen::Dynamic, 1> & x) {
        // Display Result
        std::cout << "\nAlg. Iterations: ( " << iterations << ")" << std::endl;
        std::cout << "x: ( " << x.transpose() << ")" << std::endl;
        std::cout << "f(x): ( " << fx << ")" << std::endl;
    }

    // Variables
    double finiteDiffEpsilon{};
//    CharucoBoard charucoBoard;
    LBFGSpp::LBFGSParam<Primitive> algParams{};
    std::unique_ptr<LBFGSpp::LBFGSSolver<Primitive>> solver;
};

#endif //METROLOGY2020_ALGORITHMS_EXTRINSICS_H
