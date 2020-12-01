#include "2d23d.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/core.hpp"
#include <iostream>
#include <fstream>
#include <ctime>
//#include "Common/Manifold/IO/PlyMeshReader.h"
#include "MeasurementHelper.h"


namespace BoeingMetrology
{
	cv::Rect2d currentRect;

	cv::Mat binaryTile;
	cv::Mat blendedTile;
	cv::Mat originalTile;

	cv::Mat originalMat;
	cv::Mat binaryMat;
	cv::Mat blendedMat;
	cv::Mat originalCopy;


	const std::string binaryMatName = "Binary Mat";
	const std::string blendedMatName = "Blended Mat";
	const std::string originalMatName = "Original Mat";

	const std::string binaryTileName = "Binary Tile";
	const std::string blendedTileName = "Blended Tile";
	const std::string originalTileName = "Original Tile";

	const std::string selectionWindowName = "Select Hole ROIs";

	const double inchesConversion = 39.3700787402;

	cv::Mat AddWeightedMats(cv::Mat color, cv::Mat singleChannelAdd, float colorWeight)
	{
		cv::Mat result = cv::Mat(color.rows, color.cols, CV_8UC3);
		float alpha = colorWeight;
		float beta = 1.0f - alpha;

		uchar *colorptr = color.ptr<uchar>();
		uchar *addptr = singleChannelAdd.ptr<uchar>();
		uchar* resptr = result.ptr<uchar>();
		for (int row = 0; row < color.rows; ++row)
		{
			for (int col = 0; col < color.cols; ++col)
			{
				colorptr = color.ptr<uchar>(row, col);
				addptr = singleChannelAdd.ptr<uchar>(row, col);
				resptr = result.ptr<uchar>(row, col);
				for (int cn = 0; cn < color.channels(); ++cn)
				{
					resptr[cn] = static_cast<uchar>(colorptr[cn] * alpha + addptr[0] * beta);
				}
			}
		}
		return result;
	}

	bool pointPicked = false;
	cv::Point pointOne, pointTwo;
	std::vector<cv::Point> PickedPoints;
	cv::RotatedRect currentEllipseRect = cv::RotatedRect();
	bool hasEllipse = false;

	void DrawEllipse(cv::Mat &toDrawOn, std::vector<cv::Point> pointsToFit)
	{
		cv::RotatedRect ellipseRect = cv::fitEllipse(pointsToFit);
		currentEllipseRect = ellipseRect;
		cv::ellipse(toDrawOn, ellipseRect, cv::Scalar(255, 0, 0));
	}

	cv::Point FindNearestLine(cv::Mat &binaryImg, cv::Point query, bool &hasNeighbor, double maxDist)
	{
		cv::Point nearestNeighbor = cv::Point();
		double neighborDist2 = 10000;
		hasNeighbor = false;

		uchar *binRow;
		for (int row = std::max(query.y - 5, 0); row <= std::min(query.y + 5, binaryImg.rows - 1); ++row)
		{
			binRow = binaryImg.ptr<uchar>(row);
			for (int col = std::max(0, query.x - 5); col <= std::min(query.x + 5, binaryImg.cols - 1); ++col)
			{
				if (binRow[col] > 0)
				{
					hasNeighbor = true;
					double dist2 = pow(query.x - col, 2) + pow(query.y - row, 2);
					if (dist2 < neighborDist2)
					{
						nearestNeighbor = cv::Point(col, row);
						neighborDist2 = dist2;
					}
				}
			}
		}
		if (neighborDist2 > maxDist)
		{
			hasNeighbor = false;
		}
		return nearestNeighbor;
	}

	cv::RotatedRect RansacEllipse(cv::Mat pointSource, std::vector<cv::Point> &inliers, double errorMax, int iterations, int maxIterations)
	{

		if (iterations > maxIterations)
		{
			return cv::RotatedRect();
		}
		std::srand(static_cast<unsigned int>(std::time(0)));

		int maxInliers = 0;
		std::vector<cv::Point> bestInliers = {};
		/*{
			cv::RotatedRect bestFitEllipse = cv::fitEllipse(inliers);

			cv::Mat ellipseMat = cv::Mat::zeros(pointSource.rows, pointSource.cols, CV_8UC1);
			cv::ellipse(ellipseMat, bestFitEllipse, cv::Scalar(255));

			std::vector<cv::Point> fitInliers = {};
			for(cv::Point potentialInlier : inliers)
			{
				bool isInlier = false;
				FindNearestLine(ellipseMat, potentialInlier, isInlier, errorMax);

				if (isInlier)
				{
					fitInliers.push_back(potentialInlier);
				}
			}

			if (fitInliers.size() > maxInliers)
			{
				maxInliers = fitInliers.size();
				bestInliers = fitInliers;
			}
		}*/

		for (int i = 0; i < 3; ++i)
		{
			std::vector<int> pickedIndices = {};
			
			for (int pickI = 0; pickI < 7; ++pickI)
			{
				bool alreadyPicked = true;
				while (alreadyPicked)
				{
					int pickedIndex = std::rand() % inliers.size();
					alreadyPicked = false;
					for (int picked : pickedIndices)
					{
						if (pickedIndex == picked)
						{
							alreadyPicked = true;
						}
					}
					if (!alreadyPicked)
					{
						pickedIndices.push_back(pickedIndex);
					}
				}
			}
			std::vector<cv::Point> startingPoints = {};
			for (int picked : pickedIndices)
			{
				startingPoints.push_back(inliers[picked]);
			}
			cv::RotatedRect bestFitEllipse = cv::fitEllipse(startingPoints);

			cv::Mat ellipseMat = cv::Mat::zeros(pointSource.rows, pointSource.cols, CV_8UC1);
			cv::ellipse(ellipseMat, bestFitEllipse, cv::Scalar(255));
			
			std::vector<cv::Point> fitInliers = {};
			for (cv::Point potentialInlier : inliers)
			{
				bool isInlier = false;
				FindNearestLine(ellipseMat, potentialInlier, isInlier, errorMax);

				if (isInlier)
				{
					fitInliers.push_back(potentialInlier);
				}
			}

			if ((int)fitInliers.size() > maxInliers)
			{
				maxInliers = (int)fitInliers.size();
				bestInliers = fitInliers;
			}
		}

		if (maxInliers < 20)
		{
			return RansacEllipse(pointSource, inliers, errorMax * 2, iterations + 1, maxIterations);
		}

		cv::RotatedRect result = cv::fitEllipse(bestInliers);
		inliers = bestInliers;

		return result;
	}

	std::vector<cv::Point> FindWhitePixels(cv::Mat binaryImg)
	{
		std::vector<cv::Point> result = {};
		for (int row = 0; row < binaryImg.rows; ++row)
		{
			uchar* binRow = binaryImg.ptr<uchar>(row);
			for (int col = 0; col < binaryImg.cols; ++col)
			{
				if (binRow[col] == 255)
				{
					result.push_back(cv::Point(col, row));
				}
			}
		}
		return result;
	}


	void PickPointsCallback(int event, int x, int y, int flags, void* userdata)
	{
		if (event == cv::EVENT_LBUTTONDOWN)
		{
			int imgX = x + (int)currentRect.x;
			int imgY = y + (int)currentRect.y;
			bool goodPick = false;
			cv::Point newPoint = FindNearestLine(binaryMat, cv::Point(imgX, imgY), goodPick, 10000);
			if (goodPick)
			{
				PickedPoints.push_back(newPoint);
				if (PickedPoints.size() >= 5)
				{
					blendedMat = AddWeightedMats(originalMat, binaryMat, 0.5);
					DrawEllipse(blendedMat, PickedPoints);

					cv::imshow(blendedMatName, blendedMat);
					blendedTile = blendedMat(currentRect);
					cv::imshow(blendedTileName, blendedTile);
					hasEllipse = true;
				}
				else
				{
					hasEllipse = false;
				}
			}

		}
		else if (event == cv::EVENT_RBUTTONDOWN)
		{
			if (PickedPoints.size() > 0)
			{
				PickedPoints.resize(PickedPoints.size() - 1);

				if (PickedPoints.size() >= 5)
				{
					blendedMat = AddWeightedMats(originalMat, binaryMat, 0.5);
					DrawEllipse(blendedMat, PickedPoints);

					cv::imshow(blendedMatName, blendedMat);
					blendedTile = blendedMat(currentRect);
					cv::imshow(blendedTileName, blendedTile);
				}
				else
				{
					hasEllipse = false;
					blendedMat = AddWeightedMats(originalMat, binaryMat, 0.5);

					cv::imshow(blendedMatName, blendedMat);
					blendedTile = blendedMat(currentRect);
					cv::imshow(blendedTileName, blendedTile);
				}
			}
		}
		else if (event == cv::EVENT_MOUSEMOVE)
		{
			/*if (pointPicked)
			{
				int imgX = x + currentRect.x;
				int imgY = y + currentRect.y;
				pointTwo = cv::Point(x, y);

				double alpha = 0.5;
				blendedMat = AddWeightedMats(originalMat, binaryMat, alpha);

				cv::rectangle(blendedMat, pointOne, pointTwo, cv::Scalar(150, 150, 150));
				cv::imshow(blendedMatName, blendedMat);

				blendedTile = blendedMat(currentRect);
				cv::imshow(blendedTileName, blendedTile);
				return;
			}*/
		}
	}


	void Opfor2d23d(std::string ioFolder, std::string imgFile, std::string plyFile)
	{
	//	////////////////////		PLY MESH		/////////////////////

	//	// Parse .ply into 3d mesh
	//	std::string fullPLYfile = ioFolder + plyFile;
	//	std::ifstream in(fullPLYfile);
	//	std::string plyContents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
 //       Boeing::PlyMeshReader plyMeshReader;
	//	int size = plyContents.size();
 //       Boeing::UVTriangleMesh uvTriangleMesh(plyMeshReader.LoadPly(plyContents.c_str(), size, 1.0));

	//	// Load the mesh into the octree
 //       uvTriangleMesh.LoadOctree();
 //       auto verts = uvTriangleMesh.GetUniqueVertices();
	//	for (int vert = 0; vert < verts.size(); vert += 5000)
	//	{
	//		std::cout << (cv::Point3d)verts[vert] << std::endl;
	//	}
	//	


	//	// Shift the rotation center to the center of the mesh octree
 //       cv::Point3d center = uvTriangleMesh.intOctree->GetNodeCentroid();
	//	//////////////////		END PLY MESH		/////////////////////

	//	cv::Mat inputMat = cv::imread(ioFolder + imgFile);

	//	originalMat = inputMat.clone();
	//	if (inputMat.channels() == 3)
	//	{
	//		cv::cvtColor(inputMat, inputMat, CV_BGR2GRAY);
	//	}

	//	//cv::threshold(inputMat, inputMat, 210, 255, CV_THRESH_TOZERO);
	//	cv::Scalar avg, dev;
	//	cv::meanStdDev(inputMat, avg, dev);

	//	for (int row = 0; row < inputMat.rows; ++row)
	//	{
	//		uchar *inptr = inputMat.ptr<uchar>(row);
	//		for (int col = 0; col < inputMat.cols; ++col)
	//		{
	//			if (inptr[col] > avg[0] + dev[0])
	//			{
	//				inptr[col] = std::min(3 * inptr[col], 255);
	//			}
	//		}
	//	}

	//	cv::GaussianBlur(inputMat, inputMat, cv::Size(7, 7), 1.5, 1.5);
	//	cv::Canny(inputMat, inputMat, 5, 40, 3);


	//	float alpha = 0.5f;
	//	cv::Mat multiChannelCanny;

	//	blendedMat = AddWeightedMats(originalMat, inputMat, alpha);
	//	//cv::addWeighted(originalMat, alpha, inputMat, 1.0 - alpha, 0.0, blendedMat);

	//	binaryMat = inputMat;

	//

	//	

	//	//cv::namedWindow(blendedMatName, CV_WINDOW_NORMAL);
	//	//cv::namedWindow(binaryMatName, CV_WINDOW_NORMAL);
	//	cv::namedWindow(blendedTileName, CV_WINDOW_NORMAL);
	//	cv::namedWindow(binaryTileName, CV_WINDOW_NORMAL);
	//	//cv::namedWindow(originalMatName, CV_WINDOW_NORMAL);
	//	cv::namedWindow(originalTileName, CV_WINDOW_NORMAL);
	//	

	//	//cv::imshow(blendedMatName, blendedMat);
	//	//cv::imshow(binaryMatName, binaryMat);

	//	//cv::setMouseCallback(binaryTileName, PickPointsCallback, NULL);

	//	//cv::setMouseCallback(blendedTileName, PickPointsCallback, NULL);
	//	cv::Rect2d ROI = cv::Rect2d();


	//	

	//	bool endSelection = false;

	//	
	//	std::vector<cv::RotatedRect> selectedEllipses = {};
	//	while (!endSelection)
	//	{
	//		cv::namedWindow(selectionWindowName, CV_WINDOW_NORMAL);
	//		currentRect = cv::selectROI(selectionWindowName, blendedMat, false, false);

	//		binaryTile = binaryMat(currentRect);
	//		blendedTile = blendedMat(currentRect);
	//		

	//		cv::imshow(blendedTileName, blendedTile);
	//		cv::imshow(binaryTileName, binaryTile);
	//		
	//		std::vector<cv::Point> initialInliers = FindWhitePixels(binaryTile);

	//		bool accepted = false;
	//		while (!accepted)
	//		{
	//			std::vector<cv::Point> points = initialInliers;
	//			cv::RotatedRect roiEllipse = RansacEllipse(binaryTile, points, 2, 0, 3);

	//			roiEllipse.center.x += currentRect.x;
	//			roiEllipse.center.y += currentRect.y;


	//			blendedMat = AddWeightedMats(originalMat, binaryMat, 0.5);
	//			cv::ellipse(blendedMat, roiEllipse, cv::Scalar(0, 0, 255));

	//			originalMat.copyTo(originalCopy);
	//			cv::ellipse(originalCopy, roiEllipse, cv::Scalar(0, 0, 100));

	//			blendedTile = blendedMat(currentRect);
	//			originalTile = originalCopy(currentRect);

	//			cv::imshow(originalTileName, originalTile);

	//			cv::imshow(blendedTileName, blendedTile);
	//			//cv::imshow(blendedMatName, blendedMat);


	//			char keyChar = cv::waitKey();


	//			//accept the ellipse
	//			if (keyChar == 'a' || keyChar == 'A')
	//			{
	//				selectedEllipses.push_back(roiEllipse);
	//			}
	//			if (keyChar == 'r' || keyChar == 'R')
	//			{
	//				accepted = true;
	//			}
	//			else if (keyChar == 'p' || keyChar == 'P')
	//			{
	//				accepted = true;
	//				endSelection = true;
	//			}
	//		}
	//		
	//	}

	//	blendedMat = AddWeightedMats(originalMat, binaryMat, 0.5);
	//	originalMat.copyTo(originalCopy);
	//	for (cv::RotatedRect ellipseRect : selectedEllipses)
	//	{
	//		cv::ellipse(blendedMat, ellipseRect, cv::Scalar(0, 0, 255), 5);
	//		cv::ellipse(originalCopy, ellipseRect, cv::Scalar(0, 0, 255), 4);
	//		uchar* originPtr;
	//		for (int row = ellipseRect.center.y - 3; row < ellipseRect.center.y + 4; ++row)
	//		{
	//			originPtr = originalCopy.ptr<uchar>(row);
	//			for (int col = ellipseRect.center.x - 3; col < ellipseRect.center.x + 4; ++col)
	//			{
	//				originPtr[col * 3 + 2] = 255;
	//			}
	//		}
	//		
	//	}
	//	cv::namedWindow(blendedMatName, CV_WINDOW_NORMAL);
	//	cv::imshow(blendedMatName, blendedMat);
	//	cv::imwrite(ioFolder + "output/" + "ellipseImg.png", originalCopy);

	//	cv::waitKey();
	//	cv::Mat selectedHoles = cv::Mat::zeros(binaryMat.rows, binaryMat.cols, CV_8UC1);
	//	for (cv::RotatedRect ellipseRect : selectedEllipses)
	//	{
	//		cv::ellipse(selectedHoles, ellipseRect, cv::Scalar(255));
	//	}
	//	cv::Mat finalHoles = cv::Mat::zeros(binaryMat.rows, binaryMat.cols, CV_8UC1);
	//	std::vector<cv::Point> holePoints = FindWhitePixels(selectedHoles);
	//	for (cv::Point holepoint : holePoints)
	//	{
	//		bool hasNeighbor = false;
	//		cv::Point inImg =FindNearestLine(binaryMat, holepoint, hasNeighbor, 2);
	//		if (hasNeighbor)
	//		{
	//			uchar* finptr = finalHoles.ptr<uchar>(inImg.y, inImg.x);
	//			finptr[0] = 255;
	//		}
	//	}

	//	finalHoles = selectedHoles;
	//	
	//	cv::destroyAllWindows();

	//	cv::imwrite(ioFolder + "output/"+"FinalBoundaries.png", finalHoles);

	//	inputMat = finalHoles;

	//	int maxLevels = 8;
	//	int minPoints = 1;

	//	std::fstream outputFile2;
	//	outputFile2.open(ioFolder +"output/" + "edges3d.txt", std::ios_base::out);
	//

	//	uchar *pxl = inputMat.ptr<uchar>(0);
	//	int count = 0, count2 = 0, count3 = 0, count4 = 0, faceCount = 0, multiFaceCount2 = 0;

	//	cv::Point3f checkPt2;

	//	float floatRows = inputMat.rows, floatCols = inputMat.cols;
	//	std::vector<cv::Point3d> edge3dpoints = {};




	//	cv::Mat drawingPad;
	//	std::ofstream edgeCenters;
	//	edgeCenters.open(ioFolder + "output/edgeCenters.ply", std::ios_base::out);
	//	edgeCenters << "ply" << std::endl;
	//	edgeCenters << "format ascii 1.0" << std::endl;
	//	edgeCenters << "element vertex " << selectedEllipses.size() << std::endl;
	//	edgeCenters << "property float x" << std::endl;
	//	edgeCenters << "property float y" << std::endl;
	//	edgeCenters << "property float z" << std::endl;
	//	edgeCenters << "edge_header" << std::endl;
	//	for (cv::RotatedRect selectedEllipse : selectedEllipses)
	//	{
	//		drawingPad = cv::Mat::zeros(originalMat.rows, originalMat.cols, CV_8UC1);
	//		cv::ellipse(drawingPad, selectedEllipse, cv::Scalar(255));
	//		cv::Rect bounding = selectedEllipse.boundingRect();

	//		std::vector<cv::Point3d> edgePoints;

	//		for (int row = bounding.y; row < bounding.y + bounding.height; ++row)
	//		{
	//			uchar* drawptr = drawingPad.ptr<uchar>(row);
	//			for (int col = bounding.x; col < bounding.x + bounding.width; ++col)
	//			{
	//				if (drawptr[col] > 0)
	//				{
	//					float u = float(row) / floatRows;
	//					float v = float(col) / floatCols;
	//					std::vector<Boeing::FacePoint3f> list;

	//					checkPt2 = cv::Point3f(v, u, 0.5f);

	//					//need to troubleshoot this. 
 //                       // TBD how does this differ from calling getFacesWithinSphere() with a small radius?
 //                       uvTriangleMesh.intOctree->getObjectsFromPoint(checkPt2, list);

	//					if (list.size() > 0)
	//					{
	//					
 //                           Boeing::FacePoint3f &uvpt = list[0];
	//						cv::Point3d bary = CommonCore::MeasurementHelper::barycentricCoordinates(uvpt.v1, uvpt.v2, uvpt.v3, checkPt2);
 //                           const Boeing::FacePoint3f &pt = uvTriangleMesh.GetFace3DData(uvpt.faceId);

 //                           edgePoints.push_back(CommonCore::MeasurementHelper::barycentricCombination(pt.v1, pt.v2, pt.v3, bary));
	//					}
	//				}
	//			}
	//		}

	//		double avgX = 0, avgY = 0, avgZ = 0;
	//		for (cv::Point3d p : edgePoints)
	//		{
	//			avgX += p.x;
	//			avgY += p.y;
	//			avgZ += p.z;
	//		}
	//		avgX /= edgePoints.size();
	//		avgY /= edgePoints.size();
	//		avgZ /= edgePoints.size();

	//		edgeCenters << avgX << " " << avgY << " " << avgZ << std::endl;
	//	}
	//	edgeCenters.close();
	//	for (int row = 0; row < inputMat.rows; ++row)
	//	{
	//		pxl = inputMat.ptr<uchar>(row);
	//		for (int col = 0; col < inputMat.cols; ++col)
	//		{
	//			if (*pxl > 1)
	//			{
	//				float u = float(row) / floatRows;
	//				float v = float(col) / floatCols;
 //                   std::vector<Boeing::FacePoint3f> list;

	//				checkPt2 = cv::Point3f(v, u, 0.5f);

	//				++count;

	//				//need to troubleshoot this.
 //                   // TBD how does this differ from calling getFacesWithinSphere() with a small radius?
	//				uvTriangleMesh.intOctree->getObjectsFromPoint(checkPt2, list);

	//				if (list.size() > 0)
	//				{
	//					++count2;
	//					if (list.size() > 1)
	//					{
	//						++multiFaceCount2;
	//					}

 //                       Boeing::FacePoint3f &uvpt = list[0];
 //                       cv::Point3d bary = CommonCore::MeasurementHelper::barycentricCoordinates(uvpt.v1, uvpt.v2, uvpt.v3, checkPt2);
 //                       const Boeing::FacePoint3f &pt = uvTriangleMesh.GetFace3DData(uvpt.faceId);

 //                       edge3dpoints.push_back(CommonCore::MeasurementHelper::barycentricCombination(pt.v1, pt.v2, pt.v3, bary));
	//				}
	//			}

	//			++pxl;
	//		}
	//	}

	//	outputFile2 << "ply" << std::endl;
	//	outputFile2 << "format ascii 1.0" << std::endl;
	//	outputFile2 << "element vertex " << edge3dpoints.size() << std::endl;
	//	outputFile2 << "property float x" << std::endl << "property float y" << std::endl << "property float z" << std::endl;
	//	outputFile2 << "end_header" << std::endl;
	//	for (int i = 0; i < edge3dpoints.size(); ++i)
	//	{
	//		
	//		edge3dpoints[i].x *= inchesConversion;
	//		edge3dpoints[i].y *= inchesConversion;
	//		edge3dpoints[i].z *= inchesConversion;
	//		cv::Point3d p = edge3dpoints[i];
	//		outputFile2 << p.x << " " << p.y << " " << p.z << std::endl;
	//	}
	//	outputFile2.close();
	//	std::cout << "Ellipse center UV points " << std::endl << "_________________________________________________________________________" << std::endl;
	//	
	//	for (cv::RotatedRect ellipse : selectedEllipses)
	//	{
	//		float u = ellipse.center.y / floatRows;
	//		float v = ellipse.center.x / floatCols;
	//		std::cout << "u " << u << std::endl;
	//		std::cout << "v " << v << std::endl;
	//		std::cout << "x " << ellipse.center.x << std::endl;
	//		std::cout << "y " << ellipse.center.y << std::endl;
	//	}
	//	std::cout << "End Ellipse Center UV points " << std::endl << "____________________________________________________________________" << std::endl;
	//	std::cout << "Ellipse center 3d points " << std::endl << "___________________________________________________________________________" << std::endl;
	//	std::cout << "ply" << std::endl;
	//	std::cout << "format ascii 1.0" << std::endl;
	//	std::cout << "element vertex " << selectedEllipses.size() << std::endl;
	//	std::cout << "property float x" << std::endl << "property float y" << std::endl << "property float z" << std::endl;
	//	std::cout << "end_header" << std::endl;
	//	for (cv::RotatedRect ellipse : selectedEllipses)
	//	{
	//		float u = ellipse.center.y / floatRows;
	//		float v = ellipse.center.x / floatCols;
	//		checkPt2 = cv::Point3f(v, u, 0.5f);
 //           std::vector<Boeing::FacePoint3f> list;

 //           // TBD how does this differ from calling getFacesWithinSphere() with a small radius?
	//		uvTriangleMesh.intOctree->getObjectsFromPoint(checkPt2, list);

	//		if (list.size() > 0)
	//		{
	//			if (list.size() > 1)
	//			{
	//				++multiFaceCount2;
	//			}

 //               Boeing::FacePoint3f &uvpt = list[0];
 //               cv::Point3d bary = CommonCore::MeasurementHelper::barycentricCoordinates(uvpt.v1, uvpt.v2, uvpt.v3, checkPt2);
 //               const Boeing::FacePoint3f &pt = uvTriangleMesh.GetFace3DData(uvpt.faceId);

 //               cv::Point3d outputPoint = CommonCore::MeasurementHelper::barycentricCombination(pt.v1, pt.v2, pt.v3, bary);
	//			std::cout << outputPoint.x* inchesConversion << " " << outputPoint.y* inchesConversion << " " << outputPoint.z* inchesConversion << std::endl;
	//			
	//		}

	//	}
	//	std::cout << "End ellipse center 3d points " << std::endl << "___________________________________________________________________________" << std::endl;

	//	std::cout << count << std::endl;
	//	//outputFile1.close();
	//	std::cout << " closeing the output file " << ioFolder << "output/" << "newpoint2.txt" << std::endl;
	//}

	////cv::Vec2d UndistortPixel(cv::Point2f srcPixel, Calibration::IntrinsicData intrinsics)
	////{
	////	// Undistort the pixel 
	////	// Row based
	////	float distortedX = srcPixel.x;
	////	float distortedY = srcPixel.y;

	////	float k[5] = { 0.0 };
	////	double fx, fy, ifx, ify, cx, cy;
	////	int iters = 1;
	////	cv::Mat distMat = intrinsics.distortionCoeffs;
	////	

	////	iters = 5;
	////	cv::Matx33d camMat = intrinsics.cameraMatrix;
	////	fx = camMat(0, 0);
	////	fy = camMat(1, 1);
	////	ifx = 1.0 / fx;
	////	ify = 1.0 / fy;
	////	cx = camMat(0, 2);
	////	cy = camMat(1, 2);
	////	//std::cout << "distorted x " << distortedX << " distorted y " << distortedY << std::endl;
	////	//std::cout << "fx " << fx << " fy " << fy << " cx " << cx << " cy " << cy << std::endl;


	////	double x, y, x0, y0;

	////	x = distortedX;
	////	y = distortedY;
	////	x0 = (x - cx)*ifx;
	////	x = x0;
	////	y0 = (y - cy)*ify;
	////	y = y0;

	////	for (int jj = 0; jj < iters; jj++)
	////	{
	////		double r2 = x*x + y*y;
	////		double icdist = 1. / (1 + ((k[4] * r2 + k[1])*r2 + k[0])*r2);
	////		double deltaX = 2 * k[2] * x*y + k[3] * (r2 + 2 * x*x);
	////		double deltaY = k[2] * (r2 + 2 * y*y) + 2 * k[3] * x*y;
	////		x = (x0 - deltaX)*icdist;
	////		y = (y0 - deltaY)*icdist;
	////	}

	////	return cv::Vec2d(x*fx + cx, y*fy + cy);
	////	
	////}

	////cv::Vec3d GetPixelRay(cv::Vec2d undistortedPixel,
	////	Calibration::IntrinsicData intrinsics,
	////	Calibration::ExtrinsicData extrinsics)
	////{
	////	cv::Matx44d camTransMat = cv::Matx44d::eye();
	////	for (int y = 0; y < 3; ++y)
	////	{
	////		for (int x = 0; x < 4; ++x)
	////		{
	////			camTransMat(y, x) = extrinsics.transform(y, x);
	////		}
	////	}

	////	cv::Matx33d camMat = intrinsics.cameraMatrix;
	////	cv::Vec4d pixelRay = cv::Vec4d();
	////	pixelRay(0) = (undistortedPixel[0] - camMat(0, 2)) / camMat(0, 0);
	////	pixelRay(1) = (undistortedPixel[1] - camMat(1, 2)) / camMat(1, 1);
	////	pixelRay(2) = 1;
	////	pixelRay(3) = 0;

	////	pixelRay = camTransMat*pixelRay;
	////	cv::normalize(pixelRay);
	////	
	////	return cv::Vec3d(pixelRay(0), pixelRay(1), pixelRay(2));
	////}

	////cv::Vec3d vec3dFromMat(cv::Mat mat)
	////{
	////	double *ptr = mat.ptr<double>(0);
	////	return cv::Vec3d(ptr[0], ptr[1], ptr[2]);
	////}

	////cv::Point3d ProjectMutualObservations(std::vector<CAMERA_NAME> cameras,
	////	Calibration::MultiCameraExtrinsicData extrinsics,
	////	Calibration::MultiCameraIntrinsicData intrinsics,
	////	std::vector<cv::Point2f> mutualObservations)
	////{
	////	std::vector<cv::Vec3d> rays = {};
	////	std::vector<cv::Vec3d> origins = {};

	////	for (int i = 0; i < cameras.size(); ++i)
	////	{
	////		cv::Point2i pixel = mutualObservations[i];
	////		CAMERA_NAME camera = cameras[i];

	////		cv::Vec2d undistortedPixel = UndistortPixel(pixel, intrinsics.cameraData[camera]);
	////		rays.push_back(GetPixelRay(undistortedPixel, intrinsics.cameraData[camera], extrinsics.cameraData[camera]));
	////		origins.push_back(vec3dFromMat(extrinsics.cameraData[camera].GetTranslationVector()));
	////	}

	////	cv::Matx33d A = cv::Matx33d::zeros();
	////	cv::Matx31d bMat = cv::Matx31d::zeros();


	////	cv::Matx33d eye = cv::Matx33d::eye();
	////	for (int i = 0; i < rays.size(); ++i)
	////	{
	////		A = A + (eye - rays[i] * rays[i].t());
	////		bMat = bMat + ((eye - rays[i] * rays[i].t())*origins[i]);
	////	}
	////	
	////	cv::Vec3d X;
	////	cv::solve(A, bMat, X, cv::DECOMP_SVD);

	////	return cv::Point3d(X(0), X(1), X(2));
	////}

	////FeatureBasedPointCloud::FeatureBasedPointCloud()
	////{
	////	mWorldPoints = {};
	////	mCameras = {};
	////	mCameraExtrinsics = Calibration::MultiCameraExtrinsicData();
	////	mCameraIntrinsics = Calibration::MultiCameraIntrinsicData();
	////	mMutualObservations = std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>>();
	////	mObservations = Calibration::MultiCameraObservation();
	////	mMarkerIdToPointMap = std::map<MARKER_ID, cv::Point3d*>();
	////	mHasObservationData = false;
	////}

	////FeatureBasedPointCloud::FeatureBasedPointCloud(Calibration::MultiCameraObservation observations,
	////	Calibration::MultiCameraExtrinsicData extrinsics,
	////	Calibration::MultiCameraIntrinsicData intrinsics)
	////{
	////	mObservations = observations;
	////	mHasObservationData = true;
	////	mWorldPoints = {};
	////	mCameras = {};
	////	for (auto it = mObservations.cameraObservation.begin(); it != mObservations.cameraObservation.end(); ++it)
	////	{
	////		mCameras.push_back(it->first);
	////	}
	////	mCameraExtrinsics = extrinsics;
	////	mCameraIntrinsics = intrinsics;

	////	mMutualObservations = std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>>();

	////	for (CAMERA_ID camId = 0; camId < mCameras.size(); ++camId)
	////	{
	////		Calibration::CameraObservation obs = mObservations.cameraObservation[mCameras[camId]];
	////		for (OBSERVATION_INDEX obsId = 0; obsId < obs.observedPoints.size(); ++obsId)
	////		{
	////			MARKER_ID mId = obs.markerIdentifier[obsId];
	////			if (mMutualObservations.count(mId) == 0)
	////			{
	////				mMutualObservations[mId] = {};
	////			}
	////			mMutualObservations[mId].push_back(std::pair<CAMERA_ID, OBSERVATION_INDEX>(camId, obsId));
	////		}
	////	}

	////	for (auto& markerObservations : mMutualObservations)
	////	{
	////		std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>> sharedObservations = markerObservations.second;
	////		//each marker needs to be seen by at least 2 cameras to reconstruct
	////		if (sharedObservations.size() > 1)
	////		{
	////			std::vector<cv::Point2f> sharedPoints = {};
	////			std::vector<CAMERA_NAME> camNames = {};
	////			for (auto& camObservation : sharedObservations)
	////			{
	////				sharedPoints.push_back(mObservations.cameraObservation[mCameras[camObservation.first]].observedPoints[camObservation.second]);
	////				camNames.push_back(mCameras[camObservation.first]);
	////			}
	////			cv::Vec3d projectedPoint = ProjectMutualObservations(camNames, mCameraExtrinsics, 
	////																	mCameraIntrinsics, sharedPoints);
	////			mWorldPoints.push_back(projectedPoint);
	////			mMarkerIdToPointMap[markerObservations.first] = &mWorldPoints[mWorldPoints.size() - 1];
	////		}
	////	}
	////	
	////}
	//	//	////////////////////		PLY MESH		/////////////////////

	//	// Parse .ply into 3d mesh
	//	std::string fullPLYfile = ioFolder + plyFile;
	//	std::ifstream in(fullPLYfile);
	//	std::string plyContents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
 //       Boeing::PlyMeshReader plyMeshReader;
	//	int size = plyContents.size();
 //       Boeing::UVTriangleMesh uvTriangleMesh(plyMeshReader.LoadPly(plyContents.c_str(), size, 1.0));

	//	// Load the mesh into the octree
 //       uvTriangleMesh.LoadOctree();
 //       auto verts = uvTriangleMesh.GetUniqueVertices();
	//	for (int vert = 0; vert < verts.size(); vert += 5000)
	//	{
	//		std::cout << (cv::Point3d)verts[vert] << std::endl;
	//	}
	//	


	//	// Shift the rotation center to the center of the mesh octree
 //       cv::Point3d center = uvTriangleMesh.intOctree->GetNodeCentroid();
	//	//////////////////		END PLY MESH		/////////////////////

	//	cv::Mat inputMat = cv::imread(ioFolder + imgFile);

	//	originalMat = inputMat.clone();
	//	if (inputMat.channels() == 3)
	//	{
	//		cv::cvtColor(inputMat, inputMat, CV_BGR2GRAY);
	//	}

	//	//cv::threshold(inputMat, inputMat, 210, 255, CV_THRESH_TOZERO);
	//	cv::Scalar avg, dev;
	//	cv::meanStdDev(inputMat, avg, dev);

	//	for (int row = 0; row < inputMat.rows; ++row)
	//	{
	//		uchar *inptr = inputMat.ptr<uchar>(row);
	//		for (int col = 0; col < inputMat.cols; ++col)
	//		{
	//			if (inptr[col] > avg[0] + dev[0])
	//			{
	//				inptr[col] = std::min(3 * inptr[col], 255);
	//			}
	//		}
	//	}

	//	cv::GaussianBlur(inputMat, inputMat, cv::Size(7, 7), 1.5, 1.5);
	//	cv::Canny(inputMat, inputMat, 5, 40, 3);


	//	float alpha = 0.5f;
	//	cv::Mat multiChannelCanny;

	//	blendedMat = AddWeightedMats(originalMat, inputMat, alpha);
	//	//cv::addWeighted(originalMat, alpha, inputMat, 1.0 - alpha, 0.0, blendedMat);

	//	binaryMat = inputMat;

	//

	//	

	//	//cv::namedWindow(blendedMatName, CV_WINDOW_NORMAL);
	//	//cv::namedWindow(binaryMatName, CV_WINDOW_NORMAL);
	//	cv::namedWindow(blendedTileName, CV_WINDOW_NORMAL);
	//	cv::namedWindow(binaryTileName, CV_WINDOW_NORMAL);
	//	//cv::namedWindow(originalMatName, CV_WINDOW_NORMAL);
	//	cv::namedWindow(originalTileName, CV_WINDOW_NORMAL);
	//	

	//	//cv::imshow(blendedMatName, blendedMat);
	//	//cv::imshow(binaryMatName, binaryMat);

	//	//cv::setMouseCallback(binaryTileName, PickPointsCallback, NULL);

	//	//cv::setMouseCallback(blendedTileName, PickPointsCallback, NULL);
	//	cv::Rect2d ROI = cv::Rect2d();


	//	

	//	bool endSelection = false;

	//	
	//	std::vector<cv::RotatedRect> selectedEllipses = {};
	//	while (!endSelection)
	//	{
	//		cv::namedWindow(selectionWindowName, CV_WINDOW_NORMAL);
	//		currentRect = cv::selectROI(selectionWindowName, blendedMat, false, false);

	//		binaryTile = binaryMat(currentRect);
	//		blendedTile = blendedMat(currentRect);
	//		

	//		cv::imshow(blendedTileName, blendedTile);
	//		cv::imshow(binaryTileName, binaryTile);
	//		
	//		std::vector<cv::Point> initialInliers = FindWhitePixels(binaryTile);

	//		bool accepted = false;
	//		while (!accepted)
	//		{
	//			std::vector<cv::Point> points = initialInliers;
	//			cv::RotatedRect roiEllipse = RansacEllipse(binaryTile, points, 2, 0, 3);

	//			roiEllipse.center.x += currentRect.x;
	//			roiEllipse.center.y += currentRect.y;


	//			blendedMat = AddWeightedMats(originalMat, binaryMat, 0.5);
	//			cv::ellipse(blendedMat, roiEllipse, cv::Scalar(0, 0, 255));

	//			originalMat.copyTo(originalCopy);
	//			cv::ellipse(originalCopy, roiEllipse, cv::Scalar(0, 0, 100));

	//			blendedTile = blendedMat(currentRect);
	//			originalTile = originalCopy(currentRect);

	//			cv::imshow(originalTileName, originalTile);

	//			cv::imshow(blendedTileName, blendedTile);
	//			//cv::imshow(blendedMatName, blendedMat);


	//			char keyChar = cv::waitKey();


	//			//accept the ellipse
	//			if (keyChar == 'a' || keyChar == 'A')
	//			{
	//				selectedEllipses.push_back(roiEllipse);
	//			}
	//			if (keyChar == 'r' || keyChar == 'R')
	//			{
	//				accepted = true;
	//			}
	//			else if (keyChar == 'p' || keyChar == 'P')
	//			{
	//				accepted = true;
	//				endSelection = true;
	//			}
	//		}
	//		
	//	}

	//	blendedMat = AddWeightedMats(originalMat, binaryMat, 0.5);
	//	originalMat.copyTo(originalCopy);
	//	for (cv::RotatedRect ellipseRect : selectedEllipses)
	//	{
	//		cv::ellipse(blendedMat, ellipseRect, cv::Scalar(0, 0, 255), 5);
	//		cv::ellipse(originalCopy, ellipseRect, cv::Scalar(0, 0, 255), 4);
	//		uchar* originPtr;
	//		for (int row = ellipseRect.center.y - 3; row < ellipseRect.center.y + 4; ++row)
	//		{
	//			originPtr = originalCopy.ptr<uchar>(row);
	//			for (int col = ellipseRect.center.x - 3; col < ellipseRect.center.x + 4; ++col)
	//			{
	//				originPtr[col * 3 + 2] = 255;
	//			}
	//		}
	//		
	//	}
	//	cv::namedWindow(blendedMatName, CV_WINDOW_NORMAL);
	//	cv::imshow(blendedMatName, blendedMat);
	//	cv::imwrite(ioFolder + "output/" + "ellipseImg.png", originalCopy);

	//	cv::waitKey();
	//	cv::Mat selectedHoles = cv::Mat::zeros(binaryMat.rows, binaryMat.cols, CV_8UC1);
	//	for (cv::RotatedRect ellipseRect : selectedEllipses)
	//	{
	//		cv::ellipse(selectedHoles, ellipseRect, cv::Scalar(255));
	//	}
	//	cv::Mat finalHoles = cv::Mat::zeros(binaryMat.rows, binaryMat.cols, CV_8UC1);
	//	std::vector<cv::Point> holePoints = FindWhitePixels(selectedHoles);
	//	for (cv::Point holepoint : holePoints)
	//	{
	//		bool hasNeighbor = false;
	//		cv::Point inImg =FindNearestLine(binaryMat, holepoint, hasNeighbor, 2);
	//		if (hasNeighbor)
	//		{
	//			uchar* finptr = finalHoles.ptr<uchar>(inImg.y, inImg.x);
	//			finptr[0] = 255;
	//		}
	//	}

	//	finalHoles = selectedHoles;
	//	
	//	cv::destroyAllWindows();

	//	cv::imwrite(ioFolder + "output/"+"FinalBoundaries.png", finalHoles);

	//	inputMat = finalHoles;

	//	int maxLevels = 8;
	//	int minPoints = 1;

	//	std::fstream outputFile2;
	//	outputFile2.open(ioFolder +"output/" + "edges3d.txt", std::ios_base::out);
	//

	//	uchar *pxl = inputMat.ptr<uchar>(0);
	//	int count = 0, count2 = 0, count3 = 0, count4 = 0, faceCount = 0, multiFaceCount2 = 0;

	//	cv::Point3f checkPt2;

	//	float floatRows = inputMat.rows, floatCols = inputMat.cols;
	//	std::vector<cv::Point3d> edge3dpoints = {};




	//	cv::Mat drawingPad;
	//	std::ofstream edgeCenters;
	//	edgeCenters.open(ioFolder + "output/edgeCenters.ply", std::ios_base::out);
	//	edgeCenters << "ply" << std::endl;
	//	edgeCenters << "format ascii 1.0" << std::endl;
	//	edgeCenters << "element vertex " << selectedEllipses.size() << std::endl;
	//	edgeCenters << "property float x" << std::endl;
	//	edgeCenters << "property float y" << std::endl;
	//	edgeCenters << "property float z" << std::endl;
	//	edgeCenters << "edge_header" << std::endl;
	//	for (cv::RotatedRect selectedEllipse : selectedEllipses)
	//	{
	//		drawingPad = cv::Mat::zeros(originalMat.rows, originalMat.cols, CV_8UC1);
	//		cv::ellipse(drawingPad, selectedEllipse, cv::Scalar(255));
	//		cv::Rect bounding = selectedEllipse.boundingRect();

	//		std::vector<cv::Point3d> edgePoints;

	//		for (int row = bounding.y; row < bounding.y + bounding.height; ++row)
	//		{
	//			uchar* drawptr = drawingPad.ptr<uchar>(row);
	//			for (int col = bounding.x; col < bounding.x + bounding.width; ++col)
	//			{
	//				if (drawptr[col] > 0)
	//				{
	//					float u = float(row) / floatRows;
	//					float v = float(col) / floatCols;
	//					std::vector<Boeing::FacePoint3f> list;

	//					checkPt2 = cv::Point3f(v, u, 0.5f);

	//					//need to troubleshoot this. 
 //                       // TBD how does this differ from calling getFacesWithinSphere() with a small radius?
 //                       uvTriangleMesh.intOctree->getObjectsFromPoint(checkPt2, list);

	//					if (list.size() > 0)
	//					{
	//					
 //                           Boeing::FacePoint3f &uvpt = list[0];
	//						cv::Point3d bary = CommonCore::MeasurementHelper::barycentricCoordinates(uvpt.v1, uvpt.v2, uvpt.v3, checkPt2);
 //                           const Boeing::FacePoint3f &pt = uvTriangleMesh.GetFace3DData(uvpt.faceId);

 //                           edgePoints.push_back(CommonCore::MeasurementHelper::barycentricCombination(pt.v1, pt.v2, pt.v3, bary));
	//					}
	//				}
	//			}
	//		}

	//		double avgX = 0, avgY = 0, avgZ = 0;
	//		for (cv::Point3d p : edgePoints)
	//		{
	//			avgX += p.x;
	//			avgY += p.y;
	//			avgZ += p.z;
	//		}
	//		avgX /= edgePoints.size();
	//		avgY /= edgePoints.size();
	//		avgZ /= edgePoints.size();

	//		edgeCenters << avgX << " " << avgY << " " << avgZ << std::endl;
	//	}
	//	edgeCenters.close();
	//	for (int row = 0; row < inputMat.rows; ++row)
	//	{
	//		pxl = inputMat.ptr<uchar>(row);
	//		for (int col = 0; col < inputMat.cols; ++col)
	//		{
	//			if (*pxl > 1)
	//			{
	//				float u = float(row) / floatRows;
	//				float v = float(col) / floatCols;
 //                   std::vector<Boeing::FacePoint3f> list;

	//				checkPt2 = cv::Point3f(v, u, 0.5f);

	//				++count;

	//				//need to troubleshoot this.
 //                   // TBD how does this differ from calling getFacesWithinSphere() with a small radius?
	//				uvTriangleMesh.intOctree->getObjectsFromPoint(checkPt2, list);

	//				if (list.size() > 0)
	//				{
	//					++count2;
	//					if (list.size() > 1)
	//					{
	//						++multiFaceCount2;
	//					}

 //                       Boeing::FacePoint3f &uvpt = list[0];
 //                       cv::Point3d bary = CommonCore::MeasurementHelper::barycentricCoordinates(uvpt.v1, uvpt.v2, uvpt.v3, checkPt2);
 //                       const Boeing::FacePoint3f &pt = uvTriangleMesh.GetFace3DData(uvpt.faceId);

 //                       edge3dpoints.push_back(CommonCore::MeasurementHelper::barycentricCombination(pt.v1, pt.v2, pt.v3, bary));
	//				}
	//			}

	//			++pxl;
	//		}
	//	}

	//	outputFile2 << "ply" << std::endl;
	//	outputFile2 << "format ascii 1.0" << std::endl;
	//	outputFile2 << "element vertex " << edge3dpoints.size() << std::endl;
	//	outputFile2 << "property float x" << std::endl << "property float y" << std::endl << "property float z" << std::endl;
	//	outputFile2 << "end_header" << std::endl;
	//	for (int i = 0; i < edge3dpoints.size(); ++i)
	//	{
	//		
	//		edge3dpoints[i].x *= inchesConversion;
	//		edge3dpoints[i].y *= inchesConversion;
	//		edge3dpoints[i].z *= inchesConversion;
	//		cv::Point3d p = edge3dpoints[i];
	//		outputFile2 << p.x << " " << p.y << " " << p.z << std::endl;
	//	}
	//	outputFile2.close();
	//	std::cout << "Ellipse center UV points " << std::endl << "_________________________________________________________________________" << std::endl;
	//	
	//	for (cv::RotatedRect ellipse : selectedEllipses)
	//	{
	//		float u = ellipse.center.y / floatRows;
	//		float v = ellipse.center.x / floatCols;
	//		std::cout << "u " << u << std::endl;
	//		std::cout << "v " << v << std::endl;
	//		std::cout << "x " << ellipse.center.x << std::endl;
	//		std::cout << "y " << ellipse.center.y << std::endl;
	//	}
	//	std::cout << "End Ellipse Center UV points " << std::endl << "____________________________________________________________________" << std::endl;
	//	std::cout << "Ellipse center 3d points " << std::endl << "___________________________________________________________________________" << std::endl;
	//	std::cout << "ply" << std::endl;
	//	std::cout << "format ascii 1.0" << std::endl;
	//	std::cout << "element vertex " << selectedEllipses.size() << std::endl;
	//	std::cout << "property float x" << std::endl << "property float y" << std::endl << "property float z" << std::endl;
	//	std::cout << "end_header" << std::endl;
	//	for (cv::RotatedRect ellipse : selectedEllipses)
	//	{
	//		float u = ellipse.center.y / floatRows;
	//		float v = ellipse.center.x / floatCols;
	//		checkPt2 = cv::Point3f(v, u, 0.5f);
 //           std::vector<Boeing::FacePoint3f> list;

 //           // TBD how does this differ from calling getFacesWithinSphere() with a small radius?
	//		uvTriangleMesh.intOctree->getObjectsFromPoint(checkPt2, list);

	//		if (list.size() > 0)
	//		{
	//			if (list.size() > 1)
	//			{
	//				++multiFaceCount2;
	//			}

 //               Boeing::FacePoint3f &uvpt = list[0];
 //               cv::Point3d bary = CommonCore::MeasurementHelper::barycentricCoordinates(uvpt.v1, uvpt.v2, uvpt.v3, checkPt2);
 //               const Boeing::FacePoint3f &pt = uvTriangleMesh.GetFace3DData(uvpt.faceId);

 //               cv::Point3d outputPoint = CommonCore::MeasurementHelper::barycentricCombination(pt.v1, pt.v2, pt.v3, bary);
	//			std::cout << outputPoint.x* inchesConversion << " " << outputPoint.y* inchesConversion << " " << outputPoint.z* inchesConversion << std::endl;
	//			
	//		}

	//	}
	//	std::cout << "End ellipse center 3d points " << std::endl << "___________________________________________________________________________" << std::endl;

	//	std::cout << count << std::endl;
	//	//outputFile1.close();
	//	std::cout << " closeing the output file " << ioFolder << "output/" << "newpoint2.txt" << std::endl;
	//}

	////cv::Vec2d UndistortPixel(cv::Point2f srcPixel, Calibration::IntrinsicData intrinsics)
	////{
	////	// Undistort the pixel 
	////	// Row based
	////	float distortedX = srcPixel.x;
	////	float distortedY = srcPixel.y;

	////	float k[5] = { 0.0 };
	////	double fx, fy, ifx, ify, cx, cy;
	////	int iters = 1;
	////	cv::Mat distMat = intrinsics.distortionCoeffs;
	////	

	////	iters = 5;
	////	cv::Matx33d camMat = intrinsics.cameraMatrix;
	////	fx = camMat(0, 0);
	////	fy = camMat(1, 1);
	////	ifx = 1.0 / fx;
	////	ify = 1.0 / fy;
	////	cx = camMat(0, 2);
	////	cy = camMat(1, 2);
	////	//std::cout << "distorted x " << distortedX << " distorted y " << distortedY << std::endl;
	////	//std::cout << "fx " << fx << " fy " << fy << " cx " << cx << " cy " << cy << std::endl;


	////	double x, y, x0, y0;

	////	x = distortedX;
	////	y = distortedY;
	////	x0 = (x - cx)*ifx;
	////	x = x0;
	////	y0 = (y - cy)*ify;
	////	y = y0;

	////	for (int jj = 0; jj < iters; jj++)
	////	{
	////		double r2 = x*x + y*y;
	////		double icdist = 1. / (1 + ((k[4] * r2 + k[1])*r2 + k[0])*r2);
	////		double deltaX = 2 * k[2] * x*y + k[3] * (r2 + 2 * x*x);
	////		double deltaY = k[2] * (r2 + 2 * y*y) + 2 * k[3] * x*y;
	////		x = (x0 - deltaX)*icdist;
	////		y = (y0 - deltaY)*icdist;
	////	}

	////	return cv::Vec2d(x*fx + cx, y*fy + cy);
	////	
	////}

	////cv::Vec3d GetPixelRay(cv::Vec2d undistortedPixel,
	////	Calibration::IntrinsicData intrinsics,
	////	Calibration::ExtrinsicData extrinsics)
	////{
	////	cv::Matx44d camTransMat = cv::Matx44d::eye();
	////	for (int y = 0; y < 3; ++y)
	////	{
	////		for (int x = 0; x < 4; ++x)
	////		{
	////			camTransMat(y, x) = extrinsics.transform(y, x);
	////		}
	////	}

	////	cv::Matx33d camMat = intrinsics.cameraMatrix;
	////	cv::Vec4d pixelRay = cv::Vec4d();
	////	pixelRay(0) = (undistortedPixel[0] - camMat(0, 2)) / camMat(0, 0);
	////	pixelRay(1) = (undistortedPixel[1] - camMat(1, 2)) / camMat(1, 1);
	////	pixelRay(2) = 1;
	////	pixelRay(3) = 0;

	////	pixelRay = camTransMat*pixelRay;
	////	cv::normalize(pixelRay);
	////	
	////	return cv::Vec3d(pixelRay(0), pixelRay(1), pixelRay(2));
	////}

	////cv::Vec3d vec3dFromMat(cv::Mat mat)
	////{
	////	double *ptr = mat.ptr<double>(0);
	////	return cv::Vec3d(ptr[0], ptr[1], ptr[2]);
	////}

	////cv::Point3d ProjectMutualObservations(std::vector<CAMERA_NAME> cameras,
	////	Calibration::MultiCameraExtrinsicData extrinsics,
	////	Calibration::MultiCameraIntrinsicData intrinsics,
	////	std::vector<cv::Point2f> mutualObservations)
	////{
	////	std::vector<cv::Vec3d> rays = {};
	////	std::vector<cv::Vec3d> origins = {};

	////	for (int i = 0; i < cameras.size(); ++i)
	////	{
	////		cv::Point2i pixel = mutualObservations[i];
	////		CAMERA_NAME camera = cameras[i];

	////		cv::Vec2d undistortedPixel = UndistortPixel(pixel, intrinsics.cameraData[camera]);
	////		rays.push_back(GetPixelRay(undistortedPixel, intrinsics.cameraData[camera], extrinsics.cameraData[camera]));
	////		origins.push_back(vec3dFromMat(extrinsics.cameraData[camera].GetTranslationVector()));
	////	}

	////	cv::Matx33d A = cv::Matx33d::zeros();
	////	cv::Matx31d bMat = cv::Matx31d::zeros();


	////	cv::Matx33d eye = cv::Matx33d::eye();
	////	for (int i = 0; i < rays.size(); ++i)
	////	{
	////		A = A + (eye - rays[i] * rays[i].t());
	////		bMat = bMat + ((eye - rays[i] * rays[i].t())*origins[i]);
	////	}
	////	
	////	cv::Vec3d X;
	////	cv::solve(A, bMat, X, cv::DECOMP_SVD);

	////	return cv::Point3d(X(0), X(1), X(2));
	////}

	////FeatureBasedPointCloud::FeatureBasedPointCloud()
	////{
	////	mWorldPoints = {};
	////	mCameras = {};
	////	mCameraExtrinsics = Calibration::MultiCameraExtrinsicData();
	////	mCameraIntrinsics = Calibration::MultiCameraIntrinsicData();
	////	mMutualObservations = std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>>();
	////	mObservations = Calibration::MultiCameraObservation();
	////	mMarkerIdToPointMap = std::map<MARKER_ID, cv::Point3d*>();
	////	mHasObservationData = false;
	////}

	////FeatureBasedPointCloud::FeatureBasedPointCloud(Calibration::MultiCameraObservation observations,
	////	Calibration::MultiCameraExtrinsicData extrinsics,
	////	Calibration::MultiCameraIntrinsicData intrinsics)
	////{
	////	mObservations = observations;
	////	mHasObservationData = true;
	////	mWorldPoints = {};
	////	mCameras = {};
	////	for (auto it = mObservations.cameraObservation.begin(); it != mObservations.cameraObservation.end(); ++it)
	////	{
	////		mCameras.push_back(it->first);
	////	}
	////	mCameraExtrinsics = extrinsics;
	////	mCameraIntrinsics = intrinsics;

	////	mMutualObservations = std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>>();

	////	for (CAMERA_ID camId = 0; camId < mCameras.size(); ++camId)
	////	{
	////		Calibration::CameraObservation obs = mObservations.cameraObservation[mCameras[camId]];
	////		for (OBSERVATION_INDEX obsId = 0; obsId < obs.observedPoints.size(); ++obsId)
	////		{
	////			MARKER_ID mId = obs.markerIdentifier[obsId];
	////			if (mMutualObservations.count(mId) == 0)
	////			{
	////				mMutualObservations[mId] = {};
	////			}
	////			mMutualObservations[mId].push_back(std::pair<CAMERA_ID, OBSERVATION_INDEX>(camId, obsId));
	////		}
	////	}

	////	for (auto& markerObservations : mMutualObservations)
	////	{
	////		std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>> sharedObservations = markerObservations.second;
	////		//each marker needs to be seen by at least 2 cameras to reconstruct
	////		if (sharedObservations.size() > 1)
	////		{
	////			std::vector<cv::Point2f> sharedPoints = {};
	////			std::vector<CAMERA_NAME> camNames = {};
	////			for (auto& camObservation : sharedObservations)
	////			{
	////				sharedPoints.push_back(mObservations.cameraObservation[mCameras[camObservation.first]].observedPoints[camObservation.second]);
	////				camNames.push_back(mCameras[camObservation.first]);
	////			}
	////			cv::Vec3d projectedPoint = ProjectMutualObservations(camNames, mCameraExtrinsics, 
	////																	mCameraIntrinsics, sharedPoints);
	////			mWorldPoints.push_back(projectedPoint);
	////			mMarkerIdToPointMap[markerObservations.first] = &mWorldPoints[mWorldPoints.size() - 1];
	////		}
	////	}
	////	
	}
	
}
