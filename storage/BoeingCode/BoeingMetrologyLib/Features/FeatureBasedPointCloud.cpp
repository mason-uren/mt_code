#include "FeatureBasedPointCloud.h"
#include "Calibration/Observation/CameraObservation.h"
#include <stdio.h>
#include <fstream>
#include <thread>

namespace
{
    template<typename T>
    void
        homogeneousToEuclidean(const cv::Mat & _X, cv::Mat & _x)
    {
        int d = _X.rows - 1;

        const cv::Mat_<T> & X_rows = _X.rowRange(0, d);
        const cv::Mat_<T> h = _X.row(d);

        const T * h_ptr = h[0], *h_ptr_end = h_ptr + h.cols;
        const T * X_ptr = X_rows[0];
        T * x_ptr = _x.ptr<T>(0);
        for (; h_ptr != h_ptr_end; ++h_ptr, ++X_ptr, ++x_ptr)
        {
            const T * X_col_ptr = X_ptr;
            T * x_col_ptr = x_ptr, *x_col_ptr_end = x_col_ptr + d * _x.step1();
            for (; x_col_ptr != x_col_ptr_end; X_col_ptr += X_rows.step1(), x_col_ptr += _x.step1())
                *x_col_ptr = (*X_col_ptr) / (*h_ptr);
        }
    }

    void
        homogeneousToEuclidean(cv::InputArray _X, cv::OutputArray _x)
    {
        // src
        const cv::Mat X = _X.getMat();

        // dst
        _x.create(X.rows - 1, X.cols, X.type());
        cv::Mat x = _x.getMat();

        // type
        if (X.depth() == CV_32F)
        {
            homogeneousToEuclidean<float>(X, x);
        }
        else
        {
            homogeneousToEuclidean<double>(X, x);
        }
    }

    template <typename T>
    void ApplyRadialDistortionCameraIntrinsics(const T&focal_length_x,
        const T &focal_length_y,
        const T &principal_point_x,
        const T &principal_point_y,
        const T &k1,
        const T &k2,
        const T &k3,
        const T &p1,
        const T &p2,
        const T &normalized_x,
        const T &normalized_y,
        T *image_x,
        T* image_y)
    {
        T x = normalized_x;
        T y = normalized_y;

        T r2 = x*x + y*y;
        T r4 = r2*r2;
        T r6 = r4*r2;
        T r_coeff = 1.0f + k1 * r2 + k2*r4 + k3 * r6;
        T xd = x*r_coeff + 2.0f * p1 * x * y + p2 * (r2 + 2.0f*x*x);
        T yd = y * r_coeff + 2.0f *p2 * x * y + p1 * (r2 + 2.0f * y * y);
        *image_x = focal_length_x * xd + principal_point_x;
        *image_y = focal_length_y * yd + principal_point_y;
    }

    template<typename T> inline
        T DotProduct(const T x[3], const T y[3]) {
        return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
    }


    template<typename T> inline
        void AngleAxisRotatePoint(const T angle_axis[3], const T pt[3], T result[3]) {
        const T theta2 = DotProduct(angle_axis, angle_axis);
        if (theta2 > T(std::numeric_limits<double>::epsilon())) {
            // Away from zero, use the rodriguez formula
            //
            //   result = pt costheta +
            //            (w x pt) * sintheta +
            //            w (w . pt) (1 - costheta)
            //
            // We want to be careful to only evaluate the square root if the
            // norm of the angle_axis vector is greater than zero. Otherwise
            // we get a division by zero.
            //
            const T theta = sqrt(theta2);
            const T costheta = cos(theta);
            const T sintheta = sin(theta);
            const T theta_inverse = T(1.0) / theta;

            const T w[3] = { angle_axis[0] * theta_inverse,
                angle_axis[1] * theta_inverse,
                angle_axis[2] * theta_inverse };

            // Explicitly inlined evaluation of the cross product for
            // performance reasons.
            const T w_cross_pt[3] = { w[1] * pt[2] - w[2] * pt[1],
                w[2] * pt[0] - w[0] * pt[2],
                w[0] * pt[1] - w[1] * pt[0] };
            const T tmp =
                (w[0] * pt[0] + w[1] * pt[1] + w[2] * pt[2]) * (T(1.0) - costheta);

            result[0] = pt[0] * costheta + w_cross_pt[0] * sintheta + w[0] * tmp;
            result[1] = pt[1] * costheta + w_cross_pt[1] * sintheta + w[1] * tmp;
            result[2] = pt[2] * costheta + w_cross_pt[2] * sintheta + w[2] * tmp;
        }
        else {
            // Near zero, the first order Taylor approximation of the rotation
            // matrix R corresponding to a vector w and angle w is
            //
            //   R = I + hat(w) * sin(theta)
            //
            // But sintheta ~ theta and theta * w = angle_axis, which gives us
            //
            //  R = I + hat(w)
            //
            // and actually performing multiplication with the point pt, gives us
            // R * pt = pt + w x pt.
            //
            // Switching to the Taylor expansion near zero provides meaningful
            // derivatives when evaluated using Jets.
            //
            // Explicitly inlined evaluation of the cross product for
            // performance reasons.
            const T w_cross_pt[3] = { angle_axis[1] * pt[2] - angle_axis[2] * pt[1],
                angle_axis[2] * pt[0] - angle_axis[0] * pt[2],
                angle_axis[0] * pt[1] - angle_axis[1] * pt[0] };

            result[0] = pt[0] + w_cross_pt[0];
            result[1] = pt[1] + w_cross_pt[1];
            result[2] = pt[2] + w_cross_pt[2];
        }
    }


    cv::Point2f ComputeReprojection(
        BoeingMetrology::Calibration::ExtrinsicData extrinsics,  //rotation by angleaxis and then translation
        BoeingMetrology::Calibration::IntrinsicData intrinsics, //camera intrinsics
        cv::Point3d p) //estimated 3d coordinates) const
    {
        const float focal_lengthx = (float)intrinsics.cameraMatrix(0, 0);
        const float focal_lengthy = (float)intrinsics.cameraMatrix(1, 1);
        const float principal_point_x = (float)intrinsics.cameraMatrix(0, 2);
        const float principal_point_y = (float)intrinsics.cameraMatrix(1, 2);
        const float k1 = (float)intrinsics.distortionCoeffs.at<double>(0, 0);
        const float k2 = (float)intrinsics.distortionCoeffs.at<double>(1, 0);
        const float p1 = (float)intrinsics.distortionCoeffs.at<double>(2, 0);
        const float p2 = (float)intrinsics.distortionCoeffs.at<double>(3, 0);
        const float k3 = (float)intrinsics.distortionCoeffs.at<double>(4, 0);

        std::vector<double> rodriguez = extrinsics.GetRodriguesVector();
        float R_t[6];
        for (int i = 0; i < 6; ++i)
        {
            R_t[i] = (float)rodriguez[i];
        }
        R_t[3] = -R_t[3];
        float X[3];
        X[0] = (float)p.x;
        X[1] = (float)p.y;
        X[2] = (float)p.z;
        float x[3];
        AngleAxisRotatePoint<float>(R_t, X, x);
        x[0] += R_t[3];
        x[1] += R_t[4];
        x[2] += R_t[5];

        float xn = x[0] / x[2];
        float yn = x[1] / x[2];

        float predicted_x, predicted_y;
        ApplyRadialDistortionCameraIntrinsics(focal_lengthx,
            focal_lengthy,
            principal_point_x,
            principal_point_y,
            k1, k2, k3,
            p1, p2,
            xn, yn,
            &predicted_x,
            &predicted_y);

        return cv::Point2f(predicted_x, predicted_y);
    }

    /**
    * @brief Undistort a pixel
    *
    * @param idx 1D index of pixel
    * @param resX
    * @param resY
    * @param camMat Camera Matrix
    * @param distMat Distortion coeff
    * @param undistorted output 2d undistored ray
    *
    */


    void getRay(
        cv::Point2f undistorted,
        BoeingMetrology::Calibration::IntrinsicData intrinsics,
        BoeingMetrology::Calibration::ExtrinsicData extrinsics,

        cv::Vec3d &origin,
        cv::Vec3d &ray
        )
    {
        //we are working in a strange reference frame...
        cv::Matx41d lDirMat = cv::Matx41d(
            (undistorted.x - intrinsics.cameraMatrix(0, 2)) / intrinsics.cameraMatrix(0, 0),
            (intrinsics.cameraMatrix(1, 2) - undistorted.y) / intrinsics.cameraMatrix(1, 1),
            1.0,
            0.0
            );

        std::cout << "cx " << intrinsics.cameraMatrix(0, 2) << " cy " << intrinsics.cameraMatrix(1, 2) << std::endl;



        ray = cv::Vec3d((intrinsics.cameraMatrix*extrinsics.transform * lDirMat).val);

        ray = ray / cv::norm(ray);
        origin = extrinsics.GetTranslationVector();
    }


    /**
    * @brief Get mid point of two rays on the min
    * distance
    *
    * @param origin0 Origin point of first ray
    * @param dir0 Direction of first ray
    * @param origin1 Origin point of second ray
    * @param dir1 Direction of second ray
    * @param midPoint Return value of mid point
    *
    * @return Distance between two rays
    */
    cv::Point3d getMidPoint(
        std::vector<cv::Vec3d> origins,
        std::vector<cv::Vec3d> directions,
        float &dist
        )
    {
        std::vector<cv::Vec3d> midpoints = {};
        std::vector<float> invDists = {};
        for (int i = 0; i < origins.size() - 1; ++i)
        {
            cv::Vec3d v1 = directions[i];
            cv::Vec3d p1 = origins[i];
            for (int k = i; k < origins.size(); ++k)
            {
                cv::Vec3d v2 = directions[k];
                cv::Vec3d p2 = origins[k];


				cv::Vec3d v12 = p1 - p2;
				float v1_dot_v1  = (float)  v1.dot(v1);
				float v2_dot_v2  = (float)  v2.dot(v2);
				float v1_dot_v2  = (float)  v1.dot(v2);
				float v12_dot_v1 = (float) v12.dot(v1);
				float v12_dot_v2 = (float) v12.dot(v2);

				float denom = v1_dot_v1 * v2_dot_v2 - v1_dot_v2 * v1_dot_v2;
				float distCalc = -1.0;
				if (abs(denom) < 0.000001)
				{
					distCalc = -1.0;
					continue;
				}

				float s = (v1_dot_v2 / denom) * v12_dot_v2 - (v2_dot_v2 / denom) * v12_dot_v1;
				float t = -(v1_dot_v2 / denom) * v12_dot_v1 + (v1_dot_v1 / denom) * v12_dot_v2;
				distCalc = (float)cv::norm(p1 + s*v1 - p2 - t*v2);
				midpoints.push_back((p1 + s*v1 + p2 + t*v2) / 2.0);
				invDists.push_back(1.0f / distCalc);
			}
		}

		cv::Vec3d averageMidPoint = cv::Vec3d();
		float sumDists = 0.0;
		for (int i = 0; i < midpoints.size(); ++i)
		{
			averageMidPoint += midpoints[i] * invDists[i];
			sumDists += invDists[i];
		}
		averageMidPoint /= sumDists;
		dist = sumDists;
		return averageMidPoint;
	}

}

using BoeingMetrology::Calibration::Observation::CameraObservation;

namespace BoeingMetrology
{
    namespace Features
    {

        cv::Vec2d UndistortPixel(cv::Point2f srcPixel, Calibration::IntrinsicData intrinsics)
        {
            // Undistort the pixel
            // Row based
            float distortedX = srcPixel.x;
            float distortedY = srcPixel.y;

            float k[5] = { 0.0 };
            double fx, fy, ifx, ify, cx, cy;
            int iters = 1;
            cv::Mat distMat = intrinsics.distortionCoeffs;


            iters = 5;
            cv::Matx33d camMat = intrinsics.cameraMatrix;
            fx = camMat(0, 0);
            fy = camMat(1, 1);
            ifx = 1.0 / fx;
            ify = 1.0 / fy;
            cx = camMat(0, 2);
            cy = camMat(1, 2);
            //std::cout << "distorted x " << distortedX << " distorted y " << distortedY << std::endl;
            //std::cout << "fx " << fx << " fy " << fy << " cx " << cx << " cy " << cy << std::endl;


            double x, y, x0, y0;

            x = distortedX;
            y = distortedY;
            x0 = (x - cx)*ifx;
            x = x0;
            y0 = (y - cy)*ify;
            y = y0;

            for (int jj = 0; jj < iters; jj++)
            {
                double r2 = x*x + y*y;
                double icdist = 1. / (1 + ((k[4] * r2 + k[1])*r2 + k[0])*r2);
                double deltaX = 2 * k[2] * x*y + k[3] * (r2 + 2 * x*x);
                double deltaY = k[2] * (r2 + 2 * y*y) + 2 * k[3] * x*y;
                x = (x0 - deltaX)*icdist;
                y = (y0 - deltaY)*icdist;
            }

            return cv::Vec2d(x*fx + cx, y*fy + cy);

        }

        cv::Vec3d GetPixelRay(cv::Vec2d undistortedPixel,
            Calibration::IntrinsicData intrinsics,
            Calibration::ExtrinsicData extrinsics)
        {
            cv::Matx44d camTransMat = cv::Matx44d::eye();
            for (int y = 0; y < 3; ++y)
            {
                for (int x = 0; x < 4; ++x)
                {
                    camTransMat(y, x) = extrinsics.transform(y, x);
                }
            }

            cv::Matx33d camMat = intrinsics.cameraMatrix;
            cv::Vec4d pixelRay = cv::Vec4d();
            pixelRay(0) = (undistortedPixel[0] - camMat(0, 2)) / camMat(0, 0);
            pixelRay(1) = (camMat(1, 2) - undistortedPixel[1]) / camMat(1, 1);
            pixelRay(2) = 1;
            pixelRay(3) = 0;

            pixelRay = camTransMat*pixelRay;
            cv::normalize(pixelRay);

            return cv::Vec3d(pixelRay(0), pixelRay(1), pixelRay(2));
        }

        cv::Vec3d vec3dFromMat(cv::Mat mat)
        {
            double *ptr = mat.ptr<double>(0);
            return cv::Vec3d(ptr[0], ptr[1], ptr[2]);
        }

        cv::Point3d projectObservations(std::vector<CAMERA_NAME> cameras,
            Calibration::MultiCameraExtrinsicData extrinsics,
            Calibration::MultiCameraIntrinsicData intrinsics,
            std::vector<cv::Point2f> mutualObservs)
        {
            std::vector<cv::Vec4d> A;
            std::vector<double> B;
            for (int i = 0; i < mutualObservs.size(); ++i)
            {
                CAMERA_NAME camera = cameras[i];
                cv::Point2f pixel = mutualObservs[i];

                cv::Point2f undistorted = intrinsics.cameraData[camera].UndistortPixel(pixel);
                cv::Matx34d prMat = intrinsics.cameraData[camera].cameraMatrix * extrinsics.cameraData[camera].transform;
                cv::Vec4d aRow1 = undistorted.x * cv::Vec4d(prMat(2, 0), prMat(2, 1), prMat(2, 2), prMat(2, 3)) - cv::Vec4d(prMat(0, 0), prMat(0, 1), prMat(0, 2), prMat(0, 3));
                cv::Vec4d aRow2 = undistorted.y * cv::Vec4d(prMat(2, 0), prMat(2, 1), prMat(2, 2), prMat(2, 3)) - cv::Vec4d(prMat(1, 0), prMat(1, 1), prMat(1, 2), prMat(1, 3));
                double bRow1 = 0;
                double bRow2 = 0;
                A.push_back(aRow1);
                A.push_back(aRow2);
                B.push_back(bRow1);
                B.push_back(bRow2);
            }
            A.push_back(cv::Vec4d(0, 0, 0, 1));
            B.push_back(1);

            cv::Mat aMat = cv::Mat((int)A.size(), 4, CV_32FC1);
            cv::Mat bMat = cv::Mat((int)B.size(), 1, CV_32FC1);

            for (int i = 0; i < A.size(); ++i)
            {
                float *aptr = aMat.ptr<float>(i);
                for (int col = 0; col < 4; ++col)
                {
                    aptr[col] = (float)A[i][col];
                }
                float *bptr = bMat.ptr<float>(i);
                bptr[0] = (float)B[i];
            }

            cv::Mat result;

            cv::solve(aMat, bMat, result, cv::DECOMP_SVD);

            result = result.reshape(1, 1);
            result /= result.at<float>(0, 3);

            float *resptr = result.ptr<float>(0);
            return cv::Point3d(resptr[0], resptr[1], resptr[2]);
        }

        cv::Point2f reprojectPoint(double *Rt, double* k, double fx, double cx, double fy, double cy, cv::Point3d p)
        {
            cv::Point2f result = cv::Point2f();
            cv::Matx33d matTilt = cv::Matx33d::eye();
            double X = p.x, Y = p.y, Z = p.z;

            double x = Rt[0] * X + Rt[1] * Y + Rt[2] * Z + Rt[3];
            double y = Rt[4] * X + Rt[5] * Y + Rt[6] * Z + Rt[7];
            double z = Rt[8] * X + Rt[9] * Y + Rt[10] * Z + Rt[11];
            double r2, r4, r6, a1, a2, a3, cdist, icdist2;
            double xd, yd, xd0, yd0, invProj;
            cv::Vec3d vecTilt;
            cv::Vec3d dVecTilt;
            cv::Matx22d dMatTilt;
            cv::Vec2d dXdYd;

            z = z ? 1. / z : 1;
            x *= z; y *= z;

            r2 = x*x + y*y;
            r4 = r2*r2;
            r6 = r4*r2;
            a1 = 2 * x*y;
            a2 = r2 + 2 * x*x;
            a3 = r2 + 2 * y*y;

            cdist = 1 + k[0] * r2 + k[1] * r4 + k[4] * r6;

            icdist2 = 1. / (1 + 0 * r2 + 0 * r4 + 0 * r6);
            xd0 = x*cdist*icdist2 + k[2] * a1 + k[3] * a2 + 0 * r2 + 0 * r4;
            yd0 = y*cdist*icdist2 + k[2] * a3 + k[3] * a1 + 0 * r2 + 0 * r4;

            // additional distortion by projecting onto a tilt plane
            vecTilt = matTilt*cv::Vec3d(xd0, yd0, 1);
            invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;
            xd = invProj * vecTilt(0);
            yd = invProj * vecTilt(1);

            result.x = float(xd*fx + cx);
            result.y = float(yd*fy + cy);

            return result;
        }

        cv::Point3d ProjectMutualObservations(std::vector<CAMERA_NAME> cameras,
            Calibration::MultiCameraExtrinsicData extrinsics,
            Calibration::MultiCameraIntrinsicData intrinsics,
            std::vector<cv::Point2f> mutualObservations,
            bool &isValid)
        {
            std::vector<cv::Vec4d> A;
            std::vector<double> B;
            for (int i = 0; i < mutualObservations.size(); ++i)
            {
                CAMERA_NAME camera = cameras[i];
                cv::Point2f pixel = mutualObservations[i];

                cv::Point2f undistorted = intrinsics.cameraData[camera].UndistortPixel(pixel);
                cv::Matx34d prMat = intrinsics.cameraData[camera].cameraMatrix * extrinsics.cameraData[camera].transform;
                cv::Vec4d aRow1 = undistorted.x * cv::Vec4d(prMat(2, 0), prMat(2, 1), prMat(2, 2), prMat(2, 3)) - cv::Vec4d(prMat(0, 0), prMat(0, 1), prMat(0, 2), prMat(0, 3));
                cv::Vec4d aRow2 = undistorted.y * cv::Vec4d(prMat(2, 0), prMat(2, 1), prMat(2, 2), prMat(2, 3)) - cv::Vec4d(prMat(1, 0), prMat(1, 1), prMat(1, 2), prMat(1, 3));
                double bRow1 = 0;
                double bRow2 = 0;
                A.push_back(aRow1);
                A.push_back(aRow2);
                B.push_back(bRow1);
                B.push_back(bRow2);
            }
            A.push_back(cv::Vec4d(0, 0, 0, 1));
            B.push_back(1);

            cv::Mat aMat = cv::Mat((int)A.size(), 4, CV_32FC1);
            cv::Mat bMat = cv::Mat((int)B.size(), 1, CV_32FC1);

            for (int i = 0; i < A.size(); ++i)
            {
                float *aptr = aMat.ptr<float>(i);
                for (int col = 0; col < 4; ++col)
                {
                    aptr[col] = (float)A[i][col];
                }
                float *bptr = bMat.ptr<float>(i);
                bptr[0] = (float)B[i];
            }

            cv::Mat result;

            cv::solve(aMat, bMat, result, cv::DECOMP_SVD);

            result = result.reshape(1, 1);
            result /= result.at<float>(0, 3);

            float *resptr = result.ptr<float>(0);
            return cv::Point3d(resptr[0], resptr[1], resptr[2]);
        }

        void FeatureBasedPointCloud::Initialize()
        {
            sourcePose = "";
            mWorldPoints = {};
            mCameras = {};
            mCameraExtrinsics = Calibration::MultiCameraExtrinsicData();
            mCameraIntrinsics = Calibration::MultiCameraIntrinsicData();
            mMutualObservations = std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>>();
            mObservations = Calibration::MultiCameraObservation();
            mMarkerIdToPointMap = std::map<MARKER_ID, cv::Point3d>();
            mMarkerIdToControlPoints = std::map<MARKER_ID, cv::Point3d>();
            mHasObservationData = false;

            mErrorMeasures = std::vector<double>(ErrorTypeCount, -1);
            mPlaneErrors = std::map<MARKER_ID, double>();
            mP2PErrors = std::map<MARKER_ID, double>();
            mReprojErrors = std::map<CAMERA_NAME, std::vector<double>>();

            mMaxPlaneErrorId = -1;
            mMaxP2PErrorId = -1;
            mMaxReprojErrorId = -1;
        }

        FeatureBasedPointCloud::FeatureBasedPointCloud()
        {
            Initialize();
        }

        FeatureBasedPointCloud::FeatureBasedPointCloud(Calibration::MultiCameraObservation observations,
            Calibration::MultiCameraExtrinsicData extrinsics,
            Calibration::MultiCameraIntrinsicData intrinsics,
            POSE_NAME pose)
        {
            Initialize();
            generateCloud(observations, extrinsics, intrinsics, pose);

        }

        FeatureBasedPointCloud::FeatureBasedPointCloud(std::vector<CAMERA_NAME> allCameras,
            std::vector<cv::Size> allCameraSizes,
            std::set<CAMERA_ID> camerasPresent,
            std::vector<std::vector<CAMERA_ID>> cameraIds,
            std::vector<std::vector<cv::Point2f>> imagePoints,
            std::vector<MARKER_ID> markerIds,
            std::vector<cv::Point3d> worldPoints,
            std::vector<cv::Point3d> controlPoints,
            Calibration::MultiCameraExtrinsicData extrinsics,
            Calibration::MultiCameraIntrinsicData intrinsics,
            POSE_NAME pose)
        {
            Initialize();
            mCameraExtrinsics = extrinsics;
            mCameraIntrinsics = intrinsics;
            mCameras = {};
            mHasObservationData = true;
            sourcePose = pose;

            std::vector<CameraObservation> observations = {};
            std::map<CAMERA_ID, CAMERA_ID> globalToLocal = std::map<CAMERA_ID, CAMERA_ID>();
            for (CAMERA_ID camId : camerasPresent)
            {
                mCameras.push_back(allCameras[camId]);
                CameraObservation newCamObservation = CameraObservation();
                newCamObservation.InitializeObservations(allCameras[camId], allCameraSizes[camId]);
                observations.push_back(newCamObservation);
                globalToLocal[camId] = (int)observations.size() - 1;
            }

            for (int i = 0; i < cameraIds.size(); ++i)
            {
                int pointId = i;
                mMutualObservations[markerIds[pointId]] = {};
                //for each camera that observed this point, add an observation point and map the mutual observations
                for (int k = 0; k < cameraIds[i].size(); ++k)
                {
                    CAMERA_ID globalId = cameraIds[i][k];
                    CAMERA_ID localId = globalToLocal[globalId];
                    observations[localId].AddObservation(markerIds[pointId], imagePoints[pointId][k], controlPoints[pointId]);
                    int observationCount = (int)observations[localId].observedPoints.size();
                    mMutualObservations[markerIds[pointId]].push_back(std::make_pair(cameraIds[i][k], observationCount - 1));
                }
                //for each of the points, add them to the world points vector, and the markerId maps
                mWorldPoints.push_back(worldPoints[pointId]);
                mMarkerIdToControlPoints[markerIds[pointId]] = controlPoints[pointId];
                mMarkerIdToPointMap[markerIds[pointId]] = worldPoints[pointId];
            }

            //add all the camera poses to the instance observations
            for (int i = 0; i < observations.size(); ++i)
            {
                mObservations.AddCameraPose(mCameras[i], observations[i]);
            }
        }

        void FeatureBasedPointCloud::generateCloud(Calibration::MultiCameraObservation observations,
            Calibration::MultiCameraExtrinsicData extrinsics,
            Calibration::MultiCameraIntrinsicData intrinsics,
            POSE_NAME pose)
        {
            sourcePose = pose;
            mObservations = observations;
            mHasObservationData = true;

            for (auto it = mObservations.cameraObservation.begin(); it != mObservations.cameraObservation.end(); ++it)
            {
                mCameras.push_back(it->first);
            }
            mCameraExtrinsics = extrinsics;
            mCameraIntrinsics = intrinsics;

            mMutualObservations = std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>>();

            std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>> initialMatches = std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>>();

            for (CAMERA_ID camId = 0; camId < mCameras.size(); ++camId)
            {
                CameraObservation obs = mObservations.cameraObservation[mCameras[camId]];
                for (OBSERVATION_INDEX obsId = 0; obsId < obs.observedPoints.size(); ++obsId)
                {
                    MARKER_ID mId = obs.markerIdentifier[obsId];
                    if (initialMatches.count(mId) == 0)
                    {
                        initialMatches[mId] = {};
                    }
                    initialMatches[mId].push_back(std::pair<CAMERA_ID, OBSERVATION_INDEX>(camId, obsId));
                    mMarkerIdToControlPoints[mId] = obs.controlPoints[obsId];
                }
            }

            for (auto initialMatch : initialMatches)
            {
                if (initialMatch.second.size() > 1)
                {
                    mMutualObservations[initialMatch.first] = initialMatch.second;
                }
            }

            for (auto& markerObservations : mMutualObservations)
            {
                MARKER_ID mid = markerObservations.first;
                std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>> sharedObservations = markerObservations.second;
                //each marker needs to be seen by at least 2 cameras to reconstruct
                if (sharedObservations.size() > 1)
                {
                    std::vector<cv::Point2f> sharedPoints = {};
                    std::vector<CAMERA_NAME> camNames = {};
                    for (auto& camObservation : sharedObservations)
                    {
                        sharedPoints.push_back(mObservations.cameraObservation[mCameras[camObservation.first]].observedPoints[camObservation.second]);
                        camNames.push_back(mCameras[camObservation.first]);
                    }
                    bool pointIsValid = false;
                    cv::Vec3d projectedPoint = ProjectMutualObservations(camNames, mCameraExtrinsics,
                        mCameraIntrinsics, sharedPoints, pointIsValid);
                    mWorldPoints.push_back(cv::Point3d(projectedPoint[0], projectedPoint[1], projectedPoint[2]));
                    mMarkerIdToPointMap[mid] = mWorldPoints.back();
                }
            }
        }

        void FeatureBasedPointCloud::computePlaneFitting()
        {
            mErrorMeasures[PlaneFittingL2Error] = 0;
            mErrorMeasures[PlaneFittingMaxError] = 0;
            mPlaneErrors = std::map<MARKER_ID, double>();
            mMaxPlaneErrorId = -1;


            const int pointCount = (int)mWorldPoints.size();
            if (pointCount > 0)
            {
                cv::Mat U(pointCount, 1, CV_64F);
                cv::Mat XYZ(pointCount, 3, CV_64F);
                int i = 0;
                for (auto idPointPair : mMarkerIdToPointMap)
                {
                    cv::Point3d p = idPointPair.second;
                    U.at<double>(i) = 1;
                    XYZ.at<double>(i, 0) = p.x;
                    XYZ.at<double>(i, 1) = p.y;
                    XYZ.at<double>(i, 2) = p.z;
                    ++i;
                }

                //std::cout << std::endl;

                //using least squares
                cv::Mat M = XYZ.t()*XYZ;
                cv::Mat R = XYZ.t()*U;
                cv::Mat a = M.inv()*R;
                //std::cout << "a = " << a << std::endl;

                //compute error
                cv::Mat ev = (XYZ*a - U) / norm(a);
                i = 0;
                for (auto idPointPair : mMarkerIdToPointMap)
                {
                    double d = ev.at<double>(i);
                    mPlaneErrors[idPointPair.first] = fabs(d);
                    mErrorMeasures[PlaneFittingL2Error] += d*d;
                    if (fabs(d) > mErrorMeasures[PlaneFittingMaxError])
                    {
                        mErrorMeasures[PlaneFittingMaxError] = fabs(d);
                        mMaxPlaneErrorId = idPointPair.first;
                    }

                    ++i;
                }


				mErrorMeasures[PlaneFittingL2Error] = sqrt(mErrorMeasures[PlaneFittingL2Error] / pointCount);
				//std::cout << std::endl;
				std::cout << "e_fit =" << mErrorMeasures[PlaneFittingL2Error] << " e_fit_max=" << mErrorMeasures[PlaneFittingMaxError] << std::endl;

				return;
			}
		}

		void FeatureBasedPointCloud::computeP2PErrors()
		{
			//check distances between reconstructed points and compare with the groundtruth
			mErrorMeasures[P2PL2Error] = 0;
			mErrorMeasures[P2PMaxError] = 0;
			mMaxP2PErrorId = -1;
			mP2PErrors = std::map<MARKER_ID, double>();

			int pointCount = (int)mWorldPoints.size();
			//for each point on the board
			for (auto idPointPair : mMarkerIdToPointMap)
			{
				int id1 = idPointPair.first;
				mP2PErrors[id1] = 0;
				//for each other point on the board
				for (auto idPointPair2 : mMarkerIdToPointMap)
				{
					int id2 = idPointPair2.first;
					if (id1 != id2)
					{
						//determine the world point distance and compare to the control point distance
						cv::Point3d p1 = idPointPair.second;
						cv::Point3d p2 = idPointPair2.second;
						double d_rec = cv::norm(p1 - p2);
						//std::cout << "Distance between markers " << idPointPair.first << " and " << idPointPair2.first << " = " << d_rec << std::endl;
						double d_gt = cv::norm(mMarkerIdToControlPoints[id1] - mMarkerIdToControlPoints[id2]);

						double e = fabs(d_rec - d_gt);
						//store the error in mP2PErrors
						mP2PErrors[id1] += e;
					}
				}

				mP2PErrors[id1] /= (pointCount - 1);

				if (mP2PErrors[id1] > mErrorMeasures[P2PMaxError])
				{
					mErrorMeasures[P2PMaxError] = mP2PErrors[id1];
					mMaxP2PErrorId = id1;
				}

				mErrorMeasures[P2PL2Error] += mP2PErrors[id1];
			}

			mErrorMeasures[P2PL2Error] /= pointCount;
			std::cout << "point to point error =" << mErrorMeasures[P2PL2Error] << std::endl;
		}

		void FeatureBasedPointCloud::computeReprojectionErrors(std::vector<std::vector<cv::Point2f>> &reprojectionPoints, std::vector<std::vector<OBSERVATION_INDEX>> &reprojectedObservations)
		{
			std::vector < std::vector<cv::Point3d>> points3d = {};
			reprojectionPoints = {};
			reprojectedObservations = {};

			mErrorMeasures[ReprojL2Error] = 0;
			mErrorMeasures[ReprojMaxError] = 0;
			mMaxReprojErrorId = -1;
			mReprojErrors = std::map<CAMERA_NAME, std::vector<double>>();
			std::cout << "Computing reprojection " << std::endl;
			for (int i = 0; i < mCameras.size(); ++i)
			{
				points3d.push_back({});
				reprojectionPoints.push_back({});
				reprojectedObservations.push_back({});
				mReprojErrors[mCameras[i]] = {};
			}

			for (auto markerObservation : mMutualObservations)
			{
				MARKER_ID marker = markerObservation.first;
				cv::Point3d p = mMarkerIdToPointMap[marker];
				if (markerObservation.second.size() < 2)
				{
					continue;
				}
				for (auto camObservation : markerObservation.second)
				{
					CAMERA_ID cam = camObservation.first;
					OBSERVATION_INDEX obsId = camObservation.second;
					points3d[cam].push_back(p);
					reprojectedObservations[cam].push_back(obsId);
				}
			}

			cv::Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F);

			for (CAMERA_ID cam = 0; cam < points3d.size(); ++cam)
			{
				CAMERA_NAME camname = mCameras[cam];
				CameraObservation* camObs = &mObservations.cameraObservation[camname];

				std::vector<double> rodriguez = mCameraExtrinsics.cameraData[camname].GetRodriguesVector();
				rodriguez[3] = -rodriguez[3];
				for (int k = 0; k < 3; k++)
				{
					rvec.at<double>(k, 0) = rodriguez[k];
					tvec.at<double>(k, 0) = rodriguez[3 + k];
				}

				std::vector<cv::Point2f> reprojectedF;
				for (int i = 0; i < points3d[cam].size(); ++i)
				{
					cv::Vec3d dist = cv::Vec3d(points3d[cam][i]) - cv::Vec3d(tvec.ptr<double>(0));
				}
				double *Rt = &mCameraExtrinsics.cameraData[camname].transform(0, 0);
				
				double *k = mCameraIntrinsics.cameraData[camname].distortionCoeffs.ptr<double>(0);
				
				double fx = mCameraIntrinsics.cameraData[camname].cameraMatrix(0, 0);
				double fy = mCameraIntrinsics.cameraData[camname].cameraMatrix(1, 1);
				double cx = mCameraIntrinsics.cameraData[camname].cameraMatrix(0, 2);
				double cy = mCameraIntrinsics.cameraData[camname].cameraMatrix(1, 2);

				for (int i = 0; i < points3d[cam].size(); ++i)
				{
					//reprojected.push_back(ComputeReprojection(mCameraExtrinsics.cameraData[camname], mCameraIntrinsics.cameraData[camname], points3d[cam][i]));
					/*cv::projectPoints(toReproject, rvec, tvec, mCameraIntrinsics.cameraData[camname].cameraMatrix, mCameraIntrinsics.cameraData[camname].distortionCoeffs, reprojected);
					reprojectionPoints[cam].push_back(reprojected[0]);*/

					//reprojectedF.push_back(reprojectPoint(rotation.ptr<float>(0), translation.ptr<float>(0), distortion.ptr<float>(0), fx, fy, cx, cy, points3d[cam][i]));
					reprojectionPoints[cam].push_back(reprojectPoint(Rt, k, fx, fy, cx, cy, points3d[cam][i]));

					//reprojectionPoints[cam].push_back(ComputeReprojection(mCameraExtrinsics.cameraData[camname], mCameraIntrinsics.cameraData[camname], points3d[cam][i]));
				}

				/*cv::projectPoints(points3d[cam], rvec, tvec, mCameraIntrinsics.cameraData[camname].cameraMatrix, mCameraIntrinsics.cameraData[camname].distortionCoeffs,
				reprojectionPoints[cam]);*/


                for (int i = 0; i < reprojectionPoints[cam].size(); ++i)
                {
                    cv::Vec2f offset = camObs->observedPoints[reprojectedObservations[cam][i]] - reprojectionPoints[cam][i];
                    double error = cv::norm(offset);
                    if (error > 1000)
                    {
                        std::cout << "Reprojection point out of line for observation " << camObs->markerIdentifier[reprojectedObservations[cam][i]] << std::endl;
                        continue;
                    }
                    mErrorMeasures[ReprojL2Error] += error;
                    if (error > mErrorMeasures[ReprojMaxError])
                    {
                        mErrorMeasures[ReprojMaxError] = error;
                        mMaxReprojErrorId = camObs->markerIdentifier[i];
                    }
                    mReprojErrors[camname].push_back(cv::norm(offset));
                }
            }

            int reprojectedPointCount = 1;
            for (int i = 1; i < reprojectedObservations.size(); ++i)
            {
                reprojectedPointCount += static_cast<int>(reprojectedObservations[i].size());
            }


			mErrorMeasures[ReprojL2Error] /= reprojectedPointCount;

			return;
		}

		void FeatureBasedPointCloud::computeErrors()
		{
			computeP2PErrors();
			computePlaneFitting();
			std::vector<std::vector<cv::Point2f>> reproj = {};
			std::vector<std::vector<OBSERVATION_INDEX>> obs = {};
			computeReprojectionErrors(reproj, obs);
		}

		void FeatureBasedPointCloud::getP2PErrors(double &maxError, int &maxErrorId, double &l2Error)
		{
			if (mMaxP2PErrorId == -1)
			{
				computeP2PErrors();
			}

			maxError = mErrorMeasures[P2PMaxError];
			maxErrorId = mMaxP2PErrorId;
			l2Error = mErrorMeasures[P2PL2Error];
			//std::cout << "P2P errors - maxError : " << maxError << " : L2Error : " << l2Error << std::endl;
		}

		void FeatureBasedPointCloud::getPlaneErrors(double &maxError, int &maxErrorId, double &l2Error)
		{
			if (mMaxPlaneErrorId == -1)
			{
				computePlaneFitting();
			}
			maxError = mErrorMeasures[PlaneFittingMaxError];
			maxErrorId = mMaxPlaneErrorId;
			l2Error = mErrorMeasures[PlaneFittingL2Error];
			//std::cout << "Plane errors - maxError : " << maxError << " : L2Error : " << l2Error << std::endl;
		}

		void FeatureBasedPointCloud::getReprojErrors(double &maxError, int &maxErrorId, double &l2Error)
		{
			if (mMaxReprojErrorId == -1)
			{
				std::vector<std::vector<cv::Point2f>> reproj = {};
				std::vector<std::vector<MARKER_ID>> ids = {};
				computeReprojectionErrors(reproj, ids);
			}
			maxError = mErrorMeasures[ReprojMaxError];
			maxErrorId = mMaxReprojErrorId;
			l2Error = mErrorMeasures[ReprojL2Error];
			//std::cout << "Reprojection errors - maxError : " << maxError << " : L2Error : " << l2Error << std::endl;
		}

		void FeatureBasedPointCloud::getError(ErrorType errorType, double &maxError, int &maxErrorId, double &l2Error)
		{
			switch (errorType)
			{
			case BoeingMetrology::Features::PlaneFittingL2Error:
				getPlaneErrors(maxError, maxErrorId, l2Error);
				break;
			case BoeingMetrology::Features::PlaneFittingMaxError:
				getPlaneErrors(maxError, maxErrorId, l2Error);
				break;
			case BoeingMetrology::Features::P2PL2Error:
				getP2PErrors(maxError, maxErrorId, l2Error);
				break;
			case BoeingMetrology::Features::P2PMaxError:
				getP2PErrors(maxError, maxErrorId, l2Error);
				break;
			case BoeingMetrology::Features::ReprojL2Error:
				getReprojErrors(maxError, maxErrorId, l2Error);
				break;
			case BoeingMetrology::Features::ReprojMaxError:
				getReprojErrors(maxError, maxErrorId, l2Error);
				break;
			default:
				std::cout << "Invalid error type " << std::endl;
				break;
			}
		}

		void FeatureBasedPointCloud::drawPoseErrors(ErrorType errorType, std::pair<POSE_NAME, std::vector<std::pair<CAMERA_NAME, cv::Mat>>> *poseErrorVis)
		{

			std::vector<std::pair<CAMERA_NAME, cv::Mat>> poseErrorMats = {};
			std::vector<std::vector<cv::Point2f>> reprojectionErrors = {};
			std::vector < std::vector<OBSERVATION_INDEX>> reprojectedObservations = {};
			if (errorType == ReprojL2Error || errorType == ReprojMaxError)
			{
				computeReprojectionErrors(reprojectionErrors, reprojectedObservations);
			}
			int camNum = -1;
			for (auto cameraObservation : mObservations.cameraObservation)
			{
				++camNum;
				CAMERA_NAME camera = cameraObservation.first;
				CameraObservation camObs = cameraObservation.second;

				cv::Mat cameraImage = cv::imread(camObs.fileNameOfObservation);
				if (cameraImage.size[0] == 0 || cameraImage.size[1] == 0)
				{
					std::cout << "WARNING: Pose image with file name \"" << camObs.fileNameOfObservation << "\" is empty or does not exist." << std::endl;
				}

				Utilities::DrawCircles(cameraImage, camObs.observedPoints, 20);
				//Utilities::DrawCircles(cameraImage, camObs.observedPoints, 1, cv::Scalar(0, 0, 255), 2);

				MARKER_ID maxErrorMarker = 0;
				double l2Error, maxError;
				if (errorType == ReprojL2Error || errorType == ReprojMaxError)
				{
					std::vector<cv::Point2f> &reprojPoints = reprojectionErrors[camNum];
					for (int reprojI = 0; reprojI < reprojPoints.size() && reprojI < camObs.observedPoints.size(); ++reprojI)
					{
						cv::Point2f reprojPoint = reprojPoints[reprojI];
						cv::Point2f observedPoint = camObs.observedPoints[reprojectedObservations[camNum][reprojI]];
						reprojPoint.x = std::min((float)cameraImage.cols - 1, reprojPoint.x);
						reprojPoint.x = std::max(0.0f, reprojPoint.x);
						reprojPoint.y = std::min((float)cameraImage.rows - 1, reprojPoint.y);
						reprojPoint.y = std::max(0.0f, reprojPoint.y);
						reprojPoints[reprojI] = reprojPoint;
						cv::line(cameraImage, reprojPoint, observedPoint, cv::Scalar(255, 0, 0), 3);
					}
					getReprojErrors(maxError, maxErrorMarker, l2Error);

				}
				else if (errorType == PlaneFittingL2Error || errorType == PlaneFittingMaxError)
				{
					getPlaneErrors(maxError, maxErrorMarker, l2Error);
				}
				else if (errorType == P2PL2Error || errorType == P2PMaxError)
				{
					getP2PErrors(maxError, maxErrorMarker, l2Error);
				}


                for (int i = 0; i < camObs.markerIdentifier.size(); ++i)
                {
                    if (camObs.markerIdentifier[i] == maxErrorMarker)
                    {
                        std::vector<cv::Point2f> badObs = { camObs.observedPoints[i] };
                        Utilities::DrawCircles(cameraImage, badObs, 50, cv::Scalar(255, 0, 0), 10);
                        break;
                    }
                }
                poseErrorMats.push_back(std::make_pair(camera, cameraImage));

            }
            *poseErrorVis = std::make_pair(sourcePose, poseErrorMats);
            return;
        }

        double FeatureBasedPointCloud::getCameraDist()
        {
            std::vector<double> dists;
            double dist2 = 0;
            int samples = int(mCameras.size() * mWorldPoints.size());
            for (CAMERA_NAME camera : mCameras)
            {
                cv::Mat pos = mCameraExtrinsics.cameraData[camera].GetTranslationVector();
                double *posptr = pos.ptr<double>();
                cv::Point3d camerapos = cv::Point3d(posptr[0], posptr[1], posptr[2]);

                for (cv::Point3d p : mWorldPoints)
                {
                    dist2 += pow(cv::norm(p - camerapos), 2) / samples;
                }
            }
            return sqrt(dist2);
        }

        void FeatureBasedPointCloud::WriteToPly(std::string fullFileName)
        {
            std::ofstream cloudFile;
            cloudFile.open(fullFileName);
            cloudFile << "ply" << std::endl;
            cloudFile << "format ascii 1.0" << std::endl;
            cloudFile << "element vertex " << mWorldPoints.size() << std::endl;
            cloudFile << "property float x" << std::endl;
            cloudFile << "property float y" << std::endl;
            cloudFile << "property float z" << std::endl;
            cloudFile << "element face " << 1 << std::endl;
            cloudFile << "property list uchar int vertex_index" << std::endl;
            cloudFile << "end_header" << std::endl;

            for (cv::Point3d p : mWorldPoints)
            {
                cloudFile << p.x << " " << p.y << " " << p.z << std::endl;
            }
            cloudFile.close();
        }


        FeatureCloudSorter::FeatureCloudSorter(ErrorType errorType)
        {
            toCompare = errorType;
        }

        bool FeatureCloudSorter::operator()(FeatureBasedPointCloud a, FeatureBasedPointCloud b)
        {
            return a.mErrorMeasures[toCompare] < b.mErrorMeasures[toCompare];
        }


        MultiPosePointCloud::MultiPosePointCloud(Calibration::MultiPoseObservations poseObservations,
            Calibration::MultiCameraIntrinsicData intrinsics,
            Calibration::MultiCameraExtrinsicData extrinsics)
        {
            mCameras = {};
            mCameraIndexMap = std::map<CAMERA_NAME, int>();
            mClouds = {};
            allWorldPoints = {};
            cloudMap = std::map<POSE_NAME, int>();
            mPoseObservations = poseObservations;

            for (auto it = poseObservations.cameraObservationCount.begin(); it != poseObservations.cameraObservationCount.end(); ++it)
            {
                mCameras.push_back(it->first);
                mCameraIndexMap[it->first] = (int)mCameras.size() - 1;
            }

            //std::vector<std::thread> generateThreads = {};
            for (auto it = poseObservations.multiCameraPose.begin(); it != poseObservations.multiCameraPose.end(); ++it)
            {
                cloudMap[it->first] = (int)mClouds.size();
                mClouds.push_back(FeatureBasedPointCloud());
                mClouds.back().generateCloud(it->second, extrinsics, intrinsics, it->first);
                /*std::thread generator = std::thread(&FeatureBasedPointCloud::generateCloud, mClouds.back(), it->second, extrinsics, intrinsics, it->first);
                generateThreads.push_back(std::move(generator));*/

            }

            for (auto cloud : mClouds)
            {
                for (cv::Point3d p : cloud.mWorldPoints)
                {
                    allWorldPoints.push_back(p);
                }
            }
            return;
        }

        MultiPosePointCloud::MultiPosePointCloud()
        {
            mCameras = {};
            mCameraIndexMap = std::map<CAMERA_NAME, int>();
            mClouds = {};
            allWorldPoints = {};
            cloudMap = std::map<POSE_NAME, int>();
            mPoseObservations = Calibration::MultiPoseObservations();
        }

        void MultiPosePointCloud::addCloud(FeatureBasedPointCloud cloud)
        {
            mClouds.push_back(cloud);
            mPoseObservations.multiCameraPose[cloud.sourcePose] = cloud.mObservations;
            cloudMap[cloud.sourcePose] = (int)mClouds.size() - 1;

            for (CAMERA_NAME camera : cloud.mCameras)
            {
                if (mCameraIndexMap.count(camera) == 0)
                {
                    mCameras.push_back(camera);
                    mCameraIndexMap[camera] = (int)mCameras.size() - 1;
                }
            }

            for (auto point : cloud.mWorldPoints)
            {
                allWorldPoints.push_back(point);
            }
        }


		std::vector<POSE_NAME>  MultiPosePointCloud::sortCloudsByParam(ErrorType param)
		{
			std::cout << "sorting clouds by error level " << std::endl;
			std::vector<POSE_NAME> result = {};
			for (int i = 0; i < mClouds.size(); ++i)
			{
				FeatureBasedPointCloud &cloud = mClouds[i];
				double maxError, l2Error;
				int maxId;
				switch (param)
				{
				case BoeingMetrology::Features::PlaneFittingL2Error:
					cloud.getPlaneErrors(maxError, maxId, l2Error);
					break;
				case BoeingMetrology::Features::PlaneFittingMaxError:
					cloud.getPlaneErrors(maxError, maxId, l2Error);
					break;
				case BoeingMetrology::Features::P2PL2Error:
					cloud.getP2PErrors(maxError, maxId, l2Error);
					break;
				case BoeingMetrology::Features::P2PMaxError:
					cloud.getP2PErrors(maxError, maxId, l2Error);
					break;
				case BoeingMetrology::Features::ReprojL2Error:
					cloud.getReprojErrors(maxError, maxId, l2Error);
					break;
				case BoeingMetrology::Features::ReprojMaxError:
					cloud.getReprojErrors(maxError, maxId, l2Error);
					break;
				case BoeingMetrology::Features::ErrorTypeCount:
					break;
				default:
					break;
				}
			}
			std::cout << "Initial order " << std::endl;
			std::vector<float> errorVals = {};
			for (int i = 0; i < mClouds.size(); ++i)
			{
				errorVals.push_back((float)mClouds[i].mErrorMeasures[param]);
				std::cout << mClouds[i].sourcePose << std::endl;
			}
			//std::sort(errorVals.begin(), errorVals.end());

			std::sort(mClouds.begin(), mClouds.end(), FeatureCloudSorter(param));

			std::cout << "Sorted order " << std::endl;
			//for (int i = 0; i < errorVals.size(); ++i)
			//{
			//	std::cout << errorVals[i] << std::endl;
			//	for (auto cloud : mClouds)
			//	{
			//		if (std::abs(cloud.mErrorMeasures[param] - errorVals[i]) < 0.000000001)
			//		{
			//			result.push_back(cloud.sourcePose);
			//			std::cout << cloud.sourcePose << std::endl;
			//			break;
			//		}
			//	}
			//}
			for (int i = 0; i < mClouds.size(); ++i)
			{
				result.push_back(mClouds[i].sourcePose);
				cloudMap[mClouds[i].sourcePose] = i;
				std::cout << mClouds[i].sourcePose << std::endl;
			}

			return result;
		}

		void MultiPosePointCloud::drawTopNPoses(ErrorType errorType, int nPoses,
			std::vector<std::pair<POSE_NAME, std::vector<std::pair<CAMERA_NAME, cv::Mat>>>> &posesVis)
		{
			std::vector<POSE_NAME> sortedPoses = sortCloudsByParam(errorType);

			posesVis = {};
			for (int nthPose = 0; nthPose < nPoses && nthPose < sortedPoses.size(); ++nthPose)
			{
				int cloudI = cloudMap[sortedPoses[sortedPoses.size() - nthPose - 1]];
				std::pair<POSE_NAME, std::vector<std::pair<CAMERA_NAME, cv::Mat>>> poseVis = {};
				FeatureBasedPointCloud *featureCloud = &mClouds[cloudI];
				featureCloud->drawPoseErrors(errorType, &poseVis);
				posesVis.push_back(poseVis);

				/*std::thread visThread = std::thread(&FeatureBasedPointCloud::drawPoseErrors, featureCloud, errorType, &posesVis.back());

				visThreads.push_back(std::move(visThread));*/
			}
			return;
		}

		void MultiPosePointCloud::writeAllToPlyFile(std::string fullFileName)
		{
			std::ofstream filestream;
			filestream.open(fullFileName.c_str());

			std::map<POSE_NAME, int> pointStartIndices = std::map<POSE_NAME, int>();

			std::vector<cv::Point3d> points = {};

			std::map<POSE_NAME, OBSERVATION_INDEX> topLeftIds = std::map<POSE_NAME, OBSERVATION_INDEX>();
			std::map<POSE_NAME, OBSERVATION_INDEX> topRightIds = std::map<POSE_NAME, OBSERVATION_INDEX>();
			std::map<POSE_NAME, OBSERVATION_INDEX> botLeftIds = std::map<POSE_NAME, OBSERVATION_INDEX>();
			std::map<POSE_NAME, OBSERVATION_INDEX> botRightIds = std::map<POSE_NAME, OBSERVATION_INDEX>();

			/*bool first = true;*/
			for (FeatureBasedPointCloud cloud : mClouds)
			{
				int startIndex = (int)points.size();
				pointStartIndices[cloud.sourcePose] = startIndex;
				points.insert(points.end(), cloud.mWorldPoints.begin(), cloud.mWorldPoints.end());

				/*if (first)
				{
				first = false;
				for (CAMERA_NAME camera : mCameras)
				{
				Calibration::IntrinsicData camIntrinsics = cloud.mCameraIntrinsics.cameraData[camera];
				Calibration::ExtrinsicData camExtrinsics = cloud.mCameraExtrinsics.cameraData[camera];



                std::vector<cv::Point2f> undistortedPoints = {};
                undistortedPoints.push_back(cv::Point2f(0, 0));
                undistortedPoints.push_back(cv::Point2f(0, 4911));
                undistortedPoints.push_back(cv::Point2f(7359, 0));
                undistortedPoints.push_back(cv::Point2f(7359, 4911));

                std::vector<cv::Vec3d> rays = {};
                std::vector<cv::Vec3d> origins = {};

                for (int k = 0; k < 4; ++k)
                {
                rays.push_back(cv::Vec3d());
                origins.push_back(cv::Vec3d());
                getRay(undistortedPoints[k], camIntrinsics, camExtrinsics, origins[k], rays[k]);
                }
                for (int corner = 0; corner < 4; ++corner)
                {
                for (float k = 0; k < 5.0; k += 0.01)
                {
                cv::Vec3d camPoint = origins[corner] + rays[corner] * k;
                points.push_back(camPoint);
                }
                }
                for (float k = 5.0; k < 10.0; k += 0.01)
                {
                cv::Vec3d camPoint = origins[0] + rays[0] * k;
                points.push_back(camPoint);
                }

                }
                }*/


                OBSERVATION_INDEX topleftid = 0, toprightid = 0, botleftid = 0, botrightid = 0;
                float topleftx = 100, toplefty = -1;
                float toprightx = -1, toprighty = -1;
                float botleftx = 100, botlefty = 100;
                float botrightx = -1, botrighty = 100;

                int obsId = 0;
                for (auto markerPointPair : cloud.mMarkerIdToControlPoints)
                {
                    float px = (float)markerPointPair.second.x;
                    float py = (float)markerPointPair.second.y;

                    if (px < topleftx && py > toplefty)
                    {
                        topleftx = px;
                        toplefty = py;
                        topleftid = obsId;
                    }
                    if (px > toprightx && py > toprighty)
                    {
                        toprightx = px;
                        toprighty = py;
                        toprightid = obsId;
                    }
                    if (px < botleftx && py < botlefty)
                    {
                        botleftx = px;
                        botlefty = py;
                        botleftid = obsId;
                    }
                    if (px > botrightx && py < botrighty)
                    {
                        botrightx = px;
                        botrighty = py;
                        botrightid = obsId;
                    }
                    ++obsId;
                }
                topLeftIds[cloud.sourcePose] = topleftid;
                topRightIds[cloud.sourcePose] = toprightid;
                botLeftIds[cloud.sourcePose] = botleftid;
                botRightIds[cloud.sourcePose] = botrightid;
            }

            filestream << "ply" << std::endl;
            filestream << "format ascii 1.0" << std::endl;
            filestream << "element vertex " << points.size() << std::endl;
            filestream << "property float x" << std::endl;
            filestream << "property float y" << std::endl;
            filestream << "property float z" << std::endl;
            filestream << "element face " << mClouds.size() * 2 << std::endl;
            filestream << "property list uchar int vertex_index" << std::endl;
            filestream << "end_header" << std::endl;
            for (int i = 0; i < points.size(); ++i)
            {
                filestream << points[i].x << " " << points[i].y << " " << points[i].z << std::endl;
            }

            for (int i = 0; i < mClouds.size(); ++i)
            {
                POSE_NAME pose = mClouds[i].sourcePose;
                int startid = pointStartIndices[pose];
                filestream << 3 << " " << startid + botLeftIds[pose] << " " << startid + botRightIds[pose] << " " << startid + topRightIds[pose] << std::endl;
                filestream << 3 << " " << startid + botLeftIds[pose] << " " << startid + topRightIds[pose] << " " << startid + topLeftIds[pose] << std::endl;
            }

        }

        void MultiPosePointCloud::writeParamsToFile(std::string csvFile)
        {
            std::ofstream fileStream;
            fileStream.open(csvFile);

            fileStream << "PoseName, ";
            fileStream << "P2P L2, P2P Max, P2P Max ID, ";
            fileStream << "PlaneFitting L2, PlaneFitting Max, PlaneFitting Max Id, ";
            fileStream << "Reproj L2, Reproj Max, Reproj Max Id, ";
            fileStream << "CameraDist L2" << std::endl;

            double l2Error, maxError;
            int maxErrorId;
            for (int i = 0; i < mClouds.size(); ++i)
            {
                mClouds[i].getP2PErrors(maxError, maxErrorId, l2Error);
                fileStream << mClouds[i].sourcePose << ", ";
                fileStream << l2Error << ", " << maxError << ", " << maxErrorId << ", ";

                mClouds[i].getPlaneErrors(maxError, maxErrorId, l2Error);
                fileStream << l2Error << ", " << maxError << ", " << maxErrorId << ", ";

                mClouds[i].getReprojErrors(maxError, maxErrorId, l2Error);
                fileStream << l2Error << ", " << maxError << ", " << maxErrorId << ", ";

                fileStream << mClouds[i].getCameraDist() << std::endl;
            }

            fileStream.close();
        }

    }//namespace Features
}//namespace BoeingMetrology





