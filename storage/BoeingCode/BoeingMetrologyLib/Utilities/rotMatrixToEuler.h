#include "cv.h"
#include "math.h"
#include <string>
#include <vector>
#include <map>
#include <iostream>

// From transformations.py
namespace BoeingMetrology
{
    namespace Transformations
    {
        static std::map<std::string, std::vector<int>> _AXES2TUPLE = {
            { "sxyz", { 0, 0, 0, 0 } }, { "sxyx", { 0, 0, 1, 0 } }, { "sxzy", { 0, 1, 0, 0 } },
            { "sxzx", { 0, 1, 1, 0 } }, { "syzx", { 1, 0, 0, 0 } }, { "syzy", { 1, 0, 1, 0 } },
            { "syxz", { 1, 1, 0, 0 } }, { "syxy", { 1, 1, 1, 0 } }, { "szxy", { 2, 0, 0, 0 } },
            { "szxz", { 2, 0, 1, 0 } }, { "szyx", { 2, 1, 0, 0 } }, { "szyz", { 2, 1, 1, 0 } },
            { "rzyx", { 0, 0, 0, 1 } }, { "rxyx", { 0, 0, 1, 1 } }, { "ryzx", { 0, 1, 0, 1 } },
            { "rxzx", { 0, 1, 1, 1 } }, { "rxzy", { 1, 0, 0, 1 } }, { "ryzy", { 1, 0, 1, 1 } },
            { "rzxy", { 1, 1, 0, 1 } }, { "ryxy", { 1, 1, 1, 1 } }, { "ryxz", { 2, 0, 0, 1 } },
            { "rzxz", { 2, 0, 1, 1 } }, { "rxyz", { 2, 1, 0, 1 } }, { "rzyz", { 2, 1, 1, 1 } } };
        static std::vector<int> _NEXT_AXIS = { 1, 2, 0, 1 };
        static double _EPS = 1e-8;

        // ax, ay, az in degrees
        template <typename T>
        static void euler_from_matrix(const cv::Mat & M, const std::string & order, double & ax, double & ay, double & az)
        {
            std::vector<int> args = _AXES2TUPLE[order];
            int firstaxis = args[0];
            int parity = args[1];
            int repetition = args[2];
            int frame = args[3];

            int i = firstaxis;
            int j = _NEXT_AXIS[i + parity];
            int k = _NEXT_AXIS[i - parity + 1];

            if (repetition)
            {
                double sy = std::sqrt(M.at<T>(i, j) * M.at<T>(i, j) + M.at<T>(i, k) * M.at<T>(i, k));
                if (sy > _EPS)
                {
                    ax = std::atan2(M.at<T>(i, j), M.at<T>(i, k));
                    ay = std::atan2(sy, M.at<T>(i, i));
                    az = std::atan2(M.at<T>(j, i), -M.at<T>(k, i));
                }
                else
                {
                    ax = std::atan2(-M.at<T>(j, k), M.at<T>(j, j));
                    ay = std::atan2(sy, M.at<T>(i, i));
                    az = 0.0;
                }
            }
            else
            {
                double cy = std::sqrt(M.at<T>(i, i) * M.at<T>(i, i) + M.at<T>(j, i) * M.at<T>(j, i));
                if (cy > _EPS)
                {
                    ax = std::atan2(M.at<T>(k, j), M.at<T>(k, k));
                    ay = std::atan2(-M.at<T>(k, i), cy);
                    az = std::atan2(M.at<T>(j, i), M.at<T>(i, i));
                }
                else
                {
                    ax = std::atan2(-M.at<T>(j, k), M.at<T>(j, j));
                    ay = std::atan2(-M.at<T>(k, i), cy);
                    az = 0.0;
                }
            }

            if (parity)
            {
                ax = -ax;
                ay = -ay;
                az = -az;
            }
            if (frame)
            {
                double temp = ax;
                ax = az;
                az = temp;
            }
            double radToDeg = 180.0 / 3.1415926535897932384626433832795;
            ax *= radToDeg;
            ay *= radToDeg;
            az *= radToDeg;
        }

        /*Return homogeneous rotation matrix from Euler angles and axis sequence.

        ai, aj, ak : Euler's roll, pitch and yaw angles.  UNITS ARE DEGREES.
        axes : One of 24 axis sequences as string or encoded tuple

        >> > R = euler_matrix(1, 2, 3, 'syxz')
        >> > numpy.allclose(numpy.sum(R[0]), -1.34786452)
        True
        >> > R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
        >> > numpy.allclose(numpy.sum(R[0]), -0.383436184)
        True
        >> > ai, aj, ak = (4 * math.pi) * (numpy.random.random(3) - 0.5)
        >> > for axes in _AXES2TUPLE.keys() :
        ...    R = euler_matrix(ai, aj, ak, axes)
        >> > for axes in _TUPLE2AXES.keys() :
        ...    R = euler_matrix(ai, aj, ak, axes)

        */
        template <typename T>
        static void euler_matrix(double ai, double aj, double ak, const std::string axes, cv::Mat & M)
        {
            std::vector<int> args = _AXES2TUPLE[axes];
            int firstaxis = args[0];
            int parity = args[1];
            int repetition = args[2];
            int frame = args[3];

            int i = firstaxis;
            int j = _NEXT_AXIS[i + parity];
            int k = _NEXT_AXIS[i - parity + 1];

            if (frame)
            {
                double temp = ak;
                ak = ai;
                ai = temp;

            }
            if (parity)
            {
                ai = -ai;
                aj = -aj;
                ak = -ak;
            }

            double degToRad = 3.1415926535897932384626433832795 / 180.0;
            double si = sin(ai * degToRad);
            double sj = sin(aj * degToRad);
            double sk = sin(ak * degToRad);
            double ci = cos(ai * degToRad);
            double cj = cos(aj * degToRad);
            double ck = cos(ak * degToRad);

            double cc = ci * ck;
            double cs = ci * sk;
            double sc = si * ck;
            double ss = si * sk;

            M = cv::Mat::eye(4, 4, CV_64FC1);

            if (repetition)
            {
                M.at<T>(i, i) = cj;
                M.at<T>(i, j) = sj * si;
                M.at<T>(i, k) = sj * ci;
                M.at<T>(j, i) = sj * sk;
                M.at<T>(j, j) = -cj * ss + cc;
                M.at<T>(j, k) = -cj * cs - sc;
                M.at<T>(k, i) = -sj * ck;
                M.at<T>(k, j) = cj * sc + cs;
                M.at<T>(k, k) = cj * cc - ss;
            }
            else
            {
                M.at<T>(i, i) = cj * ck;
                M.at<T>(i, j) = sj * sc - cs;
                M.at<T>(i, k) = sj * cc + ss;
                M.at<T>(j, i) = cj * sk;
                M.at<T>(j, j) = sj * ss + cc;
                M.at<T>(j, k) = sj * cs - sc;
                M.at<T>(k, i) = -sj;
                M.at<T>(k, j) = cj * si;
                M.at<T>(k, k) = cj * ci;
            }
        }

        // Rotation matrix to Euler angle
        template <typename T>
        static void TRY_ALL(const cv::Mat & M, std::map<std::string, cv::Point3d> & result)
        {
            for (auto & order : _AXES2TUPLE)
            {
                std::string blah = order.first;
                double ax, ay, az;
                euler_from_matrix<T>(M, order.first, ax, ay, az);
                result[order.first] = cv::Point3d(ax, ay, az);
                std::cout << order.first << "  " << ax << " " << ay << " " << az << std::endl;
            }
        }

        // Euler angle to rotation matrix
        template <typename T>
        static void TRY_ALL(const cv::Point3d & angles, std::map<std::string, cv::Mat> & result)
        {
            for (auto & order : _AXES2TUPLE)
            {
                std::string blah = order.first;
                cv::Mat M;
                euler_matrix<T>(angles.x, angles.y, angles.z, order.first, M);
                result[order.first] = M.clone();
            }
        }
    }
}