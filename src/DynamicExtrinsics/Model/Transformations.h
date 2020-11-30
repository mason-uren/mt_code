/**
 * @file Transformations.h
 * @brief A collection of linear transformations.
 *
 * @author Mason U'Ren
 * @email mruren@hrl.com
 */

#ifndef METROLOGY2020_ALGORITHMS_TRANSFORMATIONS_H
#define METROLOGY2020_ALGORITHMS_TRANSFORMATIONS_H

#include <cmath>

#include <Eigen/Dense>
#include <unsupported/Eigen/EulerAngles>

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

#include <Shared/DynamicExtrinsicConfig.h>
#include <Shared/Presets.h>

#include "Matrix/Matrix3d.h"


class Transformations {
public:

    /**
     * @fn void applyPans(Metrics<Primitive, Key> & extrinsic,
     *      const typename Metrics<Primitive, Key>::E_Matrix & panAngles)
     * @brief Rotate about the pan axis
     *
     * @tparam Primitive underlying Eigen::Matrix element type
     * @tparam Key Eigen::Matrix accessor type
     * @param[in,out] extrinsic camera extrinsics [N x 4 x 4]
     * @param[in] panAngles PTU pan angles (in radians) [N X 1]
     */
    template <typename Primitive = double, typename Key = std::string>
    void applyPans(typename Matrix3d<Primitive, Key>::E_Matrix & extrinsic,
                   const typename Matrix3d<Primitive, Key>::E_Matrix & panAngles) {

        assert(extrinsic.rows() == panAngles.rows());

        constexpr int ROW_0_IDX = 0;
        constexpr int ROW_2_IDX = 8;

        auto matDim{sqrt(extrinsic.cols())};
        /*
         * N x (1 x 4)
         * | * * * * |
         * | X X X X |
         * | X X X X |
         * | X X X X |
         */
        auto row_0{typename Matrix3d<Primitive, Key>::E_Matrix{extrinsic(Eigen::all, Eigen::seqN(ROW_0_IDX, matDim))}};
        /*
         * N x (1 x 4)
         * | X X X X |
         * | X X X X |
         * | * * * * |
         * | X X X X |
         */
        auto row_2{typename Matrix3d<Primitive, Key>::E_Matrix{extrinsic(Eigen::all, Eigen::seqN(ROW_2_IDX, matDim))}};
        // N x 1
        auto cosAngles{typename Matrix3d<Primitive, Key>::E_Matrix{Eigen::cos(panAngles.array())}};
        auto sinAngles{typename Matrix3d<Primitive, Key>::E_Matrix{Eigen::sin(panAngles.array())}};

        auto result{typename Matrix3d<Primitive, Key>::E_Matrix(cosAngles.asDiagonal() * row_0 + sinAngles.asDiagonal() * row_2)};

        extrinsic(Eigen::all, Eigen::seqN(ROW_2_IDX, matDim)) = -1 * sinAngles.asDiagonal() * row_0 + cosAngles.asDiagonal() * row_2;
        extrinsic(Eigen::all, Eigen::seqN(ROW_0_IDX, matDim)) = result;
    }

    /**
     * @fn void applyTilts(Metrics<Primitive, Key> & extrinsic,
     *      const typename Metrics<Primitive, Key>::E_Matrix & tiltAngles)
     * @brief Rotate about the tilt axis
     *
     * @tparam Primitive underlying dataset type
     * @tparam Key dataset accessro
     * @param[in,out] extrinsic camera extrinsics [N x 4 x 4]
     * @param[in] tiltAngles PTU tilt angles [N x 4]
     */
    template<typename Primitive = double, typename Key = std::string>
    void applyTilts(typename Matrix3d<Primitive, Key>::E_Matrix & extrinsic,
                    const typename Matrix3d<Primitive, Key>::E_Matrix & tiltAngles) {

        assert(extrinsic.rows() == tiltAngles.rows());

        constexpr int ROW_1_IDX = 4;
        constexpr int ROW_2_IDX = 8;

        auto matDim{sqrt(extrinsic.cols())};
        /*
         * N x (1 x 4)
         * | X X X X |
         * | * * * * |
         * | X X X X |
         * | X X X X |
         */
        auto row_1{typename Matrix3d<Primitive, Key>::E_Matrix{extrinsic(Eigen::all, Eigen::seqN(ROW_1_IDX, matDim))}};
        /*
         * N x (1 x 4)
         * | X X X X |
         * | X X X X |
         * | * * * * |
         * | X X X X |
         */
        auto row_2{typename Matrix3d<Primitive, Key>::E_Matrix{extrinsic(Eigen::all, Eigen::seqN(ROW_2_IDX, matDim))}};
        auto cosAngles{ typename Matrix3d<Primitive, Key>::E_Matrix{Eigen::cos(tiltAngles.array())}};
        auto sinAngles{ typename Matrix3d<Primitive, Key>::E_Matrix{Eigen::sin(tiltAngles.array())}};

        auto result{typename Matrix3d<Primitive, Key>::E_Matrix{cosAngles.asDiagonal() * row_1 - sinAngles.asDiagonal() * row_2}};

        extrinsic(Eigen::all, Eigen::seqN(ROW_2_IDX, matDim)) = sinAngles.asDiagonal() * row_1 + cosAngles.asDiagonal() * row_2;
        extrinsic(Eigen::all, Eigen::seqN(ROW_1_IDX, matDim)) = result;
    }

    /**
     * @fn void applyRigid(Metrics<Primitive, Key> & extrinsic,
     *      const typename Metrics<Primitive, Key>::E_Matrix & rotationVec,
     *      const typename Metrics<Primitive, Key>::E_Matrix & translationVec)
     * @brief 6dof transformations
     *
     * @tparam Primitive underlying dataset element type
     * @tparam Key dataset accessor type
     * @param[in,out] extrinsic camera extrinsics [N x 4 x 4]
     * @param[in] rotationVec [3 x 1]
     * @param[in] translationVec [3 x 1]
     */
    template <typename Primitive = double, typename Key = std::string>
    void applyRigid(typename Matrix3d<Primitive, Key>::E_Matrix & extrinsics,
                    const typename Matrix3d<Primitive, Key>::E_Matrix & rotationVec,
                    const typename Matrix3d<Primitive, Key>::E_Matrix & translationVec) {

        assert(rotationVec.rows() == translationVec.rows());

        // Setup: Intrinsic (XYZ), Extrinsic (ZYX)
        // Params: Intrinsice (xyz), Extrinsic (zyx)
        Eigen::EulerAngles<Primitive, DEFAULT_EULER_SYS> EulerAngle(rotationVec(2), rotationVec(1), rotationVec(0));
        auto rotationMat{EulerAngle.toRotationMatrix()};
        auto matDim{sqrt(extrinsics.cols())};

        const auto ROWS = (unsigned long long) extrinsics.rows();
        const auto COLS = (unsigned long long) extrinsics.cols();
        const auto ROT_DIM = (unsigned long long) rotationMat.rows();

        /*
         * Operation (multiply)
         *
         *  3 x 3           (3 x 4) x N
         *  | * * * |             | * * * * |
         *  | * * * |      *      | * * * * |
         *  | * * * |             | * * * * |
         *                        | X X X X |
         */
//        extrinsic(Eigen::all, Eigen::seqN(0, COLS - matDim)) = typename Metrics<Primitive, Key>::E_Map(
//            // (3 x 4) x N
//            typename Metrics<Primitive, Key>::E_Matrix{
//                // Rotation matrix N x (3 x 3) x N
//                rotationMatrices
//                *
//                // N x (3 x 4)
//                // FIXME - unable to reshape propperly...
//                //       - Sol'n is to loop values (not ideal)
//                typename Metrics<Primitive, Key>::E_Matrix{
//                        typename Metrics<Primitive, Key>::E_Map(
//                                // N x 12
//                                typename Metrics<Primitive, Key>::E_Matrix{
//                                        extrinsic(Eigen::all, Eigen::seqN(0, COLS - matDim))}.data(),
//                                ROWS * ROT_DIM, matDim
//                        )
//                }
//            }.data(), ROWS, COLS - matDim
//        );

        /*
         * Operation (add)
         *
         *  N x (3 x 1)         N x (3 x 1)
         *  | * |               | X X X * |
         *  | * |       +       | X X X * |
         *  | * |               | X X X * |
         *                      | X X X X |
         */
//        auto translationResult{typename Metrics<Primitive, Key>::E_Matrix{
//            // 3 x N
//            typename Metrics<Primitive, Key>::E_Map(
//                typename Metrics<Primitive, Key>::E_Matrix{
//                    // N x (3 x 1)
//                    typename Metrics<Primitive, Key>::E_Matrix{
//                            typename Metrics<Primitive, Key>::E_Map(
//                                    // N x 12
//                                    typename Metrics<Primitive, Key>::E_Matrix{
//                                            extrinsic(Eigen::all, Eigen::seqN(0, COLS - matDim))}.data(),
//                                    ROWS * rotationMat.cols(), matDim
//                            )
//                    }(Eigen::all, 3)
//                    +
//                    // N x (3 x 1)
//                    typename Metrics<Primitive, Key>::E_Matrix{ // +=
//                            typename Metrics<Primitive, Key>::E_Map(
//                                Metrics<Primitive, Key>::createIdentityMatrices({(unsigned long long) ROWS, 3, 3}, true)().data(),
//                                ROWS * translationVec.rows(), translationVec.rows())
//                            *
//                            translationVec}
//                }.data(), translationVec.rows(), ROWS
//            )
//        }};
//
//        extrinsic(Eigen::all, 3) = translationResult(0, Eigen::all).transpose();
//        extrinsic(Eigen::all, 7) = translationResult(1, Eigen::all).transpose();
//        extrinsic(Eigen::all, 11) = translationResult(2, Eigen::all).transpose();

//        // TODO - old
        for (auto i = 0; i < extrinsics.rows(); i++) {
            // Assumes square matrix
            typename Matrix3d<Primitive, Key>::E_Matrix unfolded{
                typename Matrix3d<Primitive, Key>::E_Map(extrinsics.row(i).data(), matDim, matDim)
            };
            /*
             * (3 x 4)
             *  | * * * * |
             *  | * * * * |
             *  | * * * * |
             *  | X X X X |
             */
            unfolded.topRows(unfolded.rows() - 1) = rotationMat * unfolded.topRows(unfolded.rows() - 1);
            /**
             * (3 x 1)
             *  | X X X * |
             *  | X X X * |
             *  | X X X * |
             *  | X X X X |
             */
            unfolded.block(0, unfolded.cols() - 1, unfolded.rows() - 1, 1) += translationVec;

            extrinsics.row(i) = typename Matrix3d<Primitive, Key>::E_Map(unfolded.data(), 1, extrinsics.cols());
        }
    }

    /**
     * @fn void applyZRigid(Metrics<Primitive, Key> & extrinsic,
     *      const typename Metrics<Primitive, Key>::E_Matrix & rotationVec,
     *      const typename Metrics<Primitive, Key>::E_Matrix & translationVec)
     * @brief Translate and rotate between pan and tilt axes
     *
     * @tparam Primitive underlying dataset element type
     * @tparam Key dataset accessor type
     * @param[in,out] extrinsic camera extrinsics [N x 4 x 4]
     * @param[in] rotationVec [3 x 1]
     * @param[in] translationVec [ 1 x 3]
     */
    template <typename Primitive = double, typename Key = std::string>
    void applyZRigid(typename Matrix3d<Primitive, Key>::E_Matrix & extrinsics,
                     const typename Matrix3d<Primitive, Key>::E_Matrix & rotationVec,
                     const typename Matrix3d<Primitive, Key>::E_Matrix & translationVec) {

        assert(rotationVec.rows() == translationVec.rows());

        // Setup: Intrinsic (XYZ), Extrinsic (ZYX)
        // Params: Intrinsice (xyz), Extrinsic (zyx)
        Eigen::EulerAngles<Primitive, DEFAULT_EULER_SYS> EulerAngles(rotationVec(0), 0, 0);
        auto rotationMat{EulerAngles.toRotationMatrix()};
        auto matDim{sqrt(extrinsics.cols())};

        const auto ROWS = (unsigned long long) extrinsics.rows();
        const auto COLS = extrinsics.cols();

        /*
         * Operation (multiply)
         *
         *  N x (3 x 3)           (3 x 4) x N
         *  | * * * |             | * * * * |
         *  | * * * |      *      | * * * * |
         *  | * * * |             | * * * * |
         *                        | X X X X |
         */
//        extrinsic(Eigen::all, Eigen::seqN(0, COLS - matDim)) = typename Metrics<Primitive, Key>::E_Map(
//                // (3 x 4) x N
//                typename Metrics<Primitive, Key>::E_Matrix{
//                        // Rotation matrix N x (3 x 3)
//                        (typename Metrics<Primitive, Key>::E_Map(
//                                Metrics<Primitive, Key>::createIdentityMatrices({(unsigned long long) ROWS, 3, 3}, true)().data(),
//                                ROWS * rotationMat.rows(), rotationMat.rows()) * rotationMat).eval()
//                        *
//                        // (3 x 4) x N
//                        typename Metrics<Primitive, Key>::E_Map(
//                                // N x 12
//                                typename Metrics<Primitive, Key>::E_Matrix{extrinsic(Eigen::all, Eigen::seqN(0, COLS - matDim))}.data(),
//                                rotationMat.cols(), matDim * ROWS
//                        )
//                }.data(),
//                ROWS, COLS - matDim
//        );

        /*
         * Operation (add)
         *
         *  N x (1 x 1)             N x 1
         *  | X X X X |      +      | * |
         *  | X X X X |
         *  | X X X * |
         *  | X X X X |
         */
//        extrinsic(Eigen::all, 11) += Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic>::Constant(ROWS, 1, translationVec(0));

        // TODO - old
        for (auto i = 0; i < extrinsics.rows(); i++) {
            // Assumes square matrix
            typename Matrix3d<Primitive, Key>::E_Matrix unfolded{
                typename Matrix3d<Primitive, Key>::E_Map(extrinsics.row(i).data(), matDim, matDim)
            };
            /*
             * (3 x 4)
             *  | * * * * |
             *  | * * * * |
             *  | * * * * |
             *  | X X X X |
             */
            unfolded.topRows(unfolded.rows() - 1) = (rotationMat * unfolded.topRows(unfolded.rows() - 1)).eval();

            /*
             * (1 x 1)
             *  | X X X X |
             *  | X X X X |
             *  | X X X * |
             *  | X X X X |
             */
            unfolded.block(2, 3, 1, 1) += translationVec;

            extrinsics.row(i) = typename Matrix3d<Primitive, Key>::E_Map(unfolded.data(), 1, extrinsics.cols());
        }
    }

    template <typename Primitive = double, typename Key = std::string>
    typename Matrix3d<Primitive, Key>::E_Matrix rotationMatToVec(typename Matrix3d<Primitive, Key>::E_Matrix & rotationMat) { //-> decltype(typename Metrics<Primitive, Key>::E_Matrix{}) {
        // Assume square matrix
        auto matDim{sqrt(rotationMat.cols())};
        typename Matrix3d<Primitive, Key>::E_Matrix reshape {
            typename Matrix3d<Primitive, Key>::E_Map(rotationMat.data(), matDim, matDim)
        };

        return typename Matrix3d<Primitive, Key>::E_Matrix{
                Eigen::Matrix3d(reshape.block(0, 0, 3, 3)).eulerAngles(2, 1, 0).reverse()
        };
    }

private:
    typedef Eigen::EulerSystemZYX DEFAULT_EULER_SYS;

    /*
     * Euler Systems Extrinsic Typdefs
     * - Negative axis rotations
     *
     *     Z   Y   X
     *     =========
     *  1) +   +   -
     *  2) +   -   +
     *  3) +   -   -
     *  4) -   +   +
     *  5) -   +   -
     *  6) -   -   +
     *  7) -   -   -
     *
     *  Note: omitting "+ + +"
     */
    typedef Eigen::EulerSystem< Eigen::EULER_Z,  Eigen::EULER_Y, -Eigen::EULER_X> EulerSystemZY_X;
    typedef Eigen::EulerSystem< Eigen::EULER_Z, -Eigen::EULER_Y,  Eigen::EULER_X> EulerSystemZ_YX;
    typedef Eigen::EulerSystem< Eigen::EULER_Z, -Eigen::EULER_Y, -Eigen::EULER_X> EulerSystemZ_Y_X;
    typedef Eigen::EulerSystem<-Eigen::EULER_Z,  Eigen::EULER_Y,  Eigen::EULER_X> EulerSystem_ZYX;
    typedef Eigen::EulerSystem<-Eigen::EULER_Z,  Eigen::EULER_Y, -Eigen::EULER_X> EulerSystem_ZY_X;
    typedef Eigen::EulerSystem<-Eigen::EULER_Z, -Eigen::EULER_Y,  Eigen::EULER_X> EulerSystem_Z_YX;
    typedef Eigen::EulerSystem<-Eigen::EULER_Z, -Eigen::EULER_Y, -Eigen::EULER_X> EulerSystem_Z_Y_X;
};


#endif //METROLOGY2020_ALGORITHMS_TRANSFORMATIONS_H
