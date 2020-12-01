#include "rochade.h"
#include "skeletonization.h"
#include "voronoi.h"

namespace BoeingMetrology
{
	namespace ImageProcessing
	{
		//downsampling, upsampling of input image im
		//this function performs downsampling if the input image width is greater than downSamplingThreshold
		//and upsampling by 2 if the image width is less than 200
		//output: 
		//the returned resized image
		//upSampleFac, downSampleFac: the returned upsampling or downsampling factor
		cv::Mat RochadeResizing(cv::Mat im, double downSamplingThreshold, double& upSampleFac, double& downSampleFac)
		{
			int imheight = im.rows;
			int imwidth = im.cols;
			upSampleFac = 1;

			cv::Mat imageInput;
			if (imwidth < 200)
			{
				imwidth *= 2;
				imheight *= 2;
				cv::resize(im, imageInput, cv::Size(imwidth, imheight), cv::INTER_LINEAR);

				upSampleFac = 2;
			}
			else
			{
				imageInput = im;
			}

			// Downsampling for large images
			downSampleFac = 1;
			while (imwidth > downSamplingThreshold)
			{
				imwidth /= 2;
				imheight /= 2;

				cv::resize(im, imageInput, cv::Size(imwidth, imheight), cv::INTER_NEAREST);
				downSampleFac *= 2;
			}

			return imageInput;
		}

		//computes the Scharr gradient for inputImage
		//output: the gradient image (CV_16U)
		cv::Mat Scharr(cv::Mat inputImage)
		{
			cv::Mat edgeImage(inputImage.size(), CV_16U);
			for (int y = 1; y < inputImage.rows - 1; y++)
			{
				for (int x = 1; x < inputImage.cols - 1; x++)
				{
					int v1 = 3 * inputImage.at<uchar>(y - 1, x - 1) - 3 * inputImage.at<uchar>(y + 1, x - 1);
					if (v1 < 0) v1 = -v1;

					int v2 = 10 * inputImage.at<uchar>(y - 1, x) - 10 * inputImage.at<uchar>(y + 1, x);
					if (v2 < 0) v2 = -v2;

					int v3 = 3 * inputImage.at<uchar>(y - 1, x + 1) - 3 * inputImage.at<uchar>(y + 1, x + 1);
					if (v3 < 0) v3 = -v3;

					int temp1 = v1 + v2 + v3;

					int h1 = 3 * inputImage.at<uchar>(y - 1, x - 1) - 3 * inputImage.at<uchar>(y - 1, x + 1);
					if (h1 < 0) h1 = -h1;

					int h2 = 10 * inputImage.at<uchar>(y, x - 1) - 10 * inputImage.at<uchar>(y, x + 1);
					if (h2 < 0) h2 = -h2;

					int h3 = 3 * inputImage.at<uchar>(y + 1, x - 1) - 3 * inputImage.at<uchar>(y + 1, x + 1);
					if (h3 < 0) h3 = -h3;

					int temp2 = h1 + h2 + h3;

					edgeImage.at<ushort>(y, x) = (ushort) sqrt(temp1*temp1 + temp2*temp2);
				}
			}

			return edgeImage;
		}

		//performs local thresholding
		//input: 
		// edgeImage: gradient image
		// localRelativeThreshold: local relative threshold
		//
		//output:
		// returned binary thresholded image
		cv::Mat RochadeLocalThresholding(cv::Mat edgeImage, double localRelativeThreshold)
		{
			if (edgeImage.channels()>1)
			{
				cv::cvtColor(edgeImage, edgeImage, cv::COLOR_RGB2GRAY);
			}

			if (edgeImage.type() != CV_16U)
			{
				edgeImage.convertTo(edgeImage, CV_16U);
			}

			//minImage = imerode(edgeImage, strel('rectangle', [9 9]));
			//maxImage = imdilate(edgeImage, strel('rectangle', [9 9]));
			cv::Mat minImage, maxImage;
			cv::Mat kernel9 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
			cv::erode(edgeImage, minImage, kernel9);
			cv::dilate(edgeImage, maxImage, kernel9);

			int imheight = edgeImage.rows;
			int imwidth = edgeImage.cols;

			//edgeMask = edgeImage > minImage + localRelativeThreshold * (maxImage - minImage);
			cv::Mat edgeMask(imheight, imwidth, CV_8U);
			for (int y = 0; y < imheight; y++)
			for (int x = 0; x < imwidth; x++)
			{
				int minv = minImage.at<ushort>(y, x);
				int maxv = maxImage.at<ushort>(y, x);
				bool b = edgeImage.at<ushort>(y, x) > minv + localRelativeThreshold*(maxv - minv);
				edgeMask.at<uchar>(y, x) = b * 255;
			}

			return edgeMask;
		}

		int pointdegree8(cv::Mat im, int x, int y)
		{
			const int cx8[9] = { -1, -1, 0, 1, 1, 1, 0, -1, -1 }, cy8[9] = { 0, -1, -1, -1, 0, 1, 1, 1, 0 };

			int cnt = 0;
			uchar b = 0;
			for (int ct = 0; ct < 9; ct++)
			{
				int i = y + cy8[ct];
				int j = x + cx8[ct];
				if (i >= 0 && i < im.rows && j >= 0 && j < im.cols)
				{
					if (ct > 0)
					{
						if (b == 0 && im.at<uchar>(i, j) > 0) cnt++;
					}
					b = im.at<uchar>(i, j);
				}
				else
				{
					b = 0;
				}
			}

			return cnt;
		}

		//removes all open edges in an input edge map skeletonMask
		//returns the binary image of the new edge map
		cv::Mat removeNonloop(cv::Mat skeletonMask)
		{
			cv::Mat loopMask;
			skeletonMask.copyTo(loopMask);

			std::list<cv::Point> endpoint;

			const int cx8[8] = { -1, -1, 0, 1, 1, 1, 0, -1 }, cy8[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
			/* extract end points*/
			int imwidth = skeletonMask.cols;
			int imheight = skeletonMask.rows;
			for (int y = 0; y < imheight; y++)
			{
				for (int x = 0; x < imwidth; x++)
				{
					if (loopMask.at<uchar>(y, x) > 0 && pointdegree8(loopMask, x, y) <= 1)
					{
						endpoint.push_back(cv::Point(x, y));
					}
				}
			}

			while (!endpoint.empty())
			{
				cv::Point p = endpoint.front();
				int x = p.x;
				int y = p.y;
				endpoint.pop_front();

				loopMask.at<uchar>(y, x) = 0;

				for (int ct = 0; ct < 8; ct++)
				{
					int i = y + cy8[ct];
					int j = x + cx8[ct];
					if (i >= 0 && i < imheight && j >= 0 && j < imwidth)
					if (loopMask.at<uchar>(i, j) > 0 && pointdegree8(loopMask, j, i) <= 1)
					{
						endpoint.push_back(cv::Point(j, i));
					}
				}
			}

			return loopMask;
		}

		//finds intersection points (with 3 or more connected edge branches) in an input edge map skeletonMask
		//returns a binary mask of the intersection points
		cv::Mat findSaddlePointCandidates(cv::Mat skeletonMask)
		{
			cv::Mat saddleCandidates = cv::Mat::zeros(skeletonMask.size(), CV_8U);

			int imwidth = skeletonMask.cols;
			int imheight = skeletonMask.rows;

			for (int y = 0; y < imheight; y++)
			{
				for (int x = 0; x < imwidth; x++)
				{
					if (skeletonMask.at<uchar>(y, x)>0 && pointdegree8(skeletonMask, x, y) >= 3)
					{
						saddleCandidates.at<uchar>(y, x) = 255;
					}
				}
			}

			return saddleCandidates;
		}

		//Closes notches and holes inside a mask
		//Input:
		// inputMask: input binary image
		// halfSize: half size of the dilation kernel
		// minNrSetPixels: a pixel in the input mask is dilated  if more than a certain number of pixels is set within the kernel.
		// foregroundLevel: the output value of foreground pixels in the output mask
		//
		// output: returned dilated binary mask
		cv::Mat fillHolesAndNotches(cv::Mat inputMask, int halfSize, int minNrSetPixels, int foregroundLevel)
		{
			if (inputMask.channels() > 1)
			{
				cv::cvtColor(inputMask, inputMask, cv::COLOR_RGB2GRAY);
			}

			cv::Mat dilatedMask;

			inputMask.copyTo(dilatedMask);

			int height = inputMask.rows;
			int width = inputMask.cols;

			for (int y = 0; y < height; y++)
			{
				for (int x = 0; x < width; x++)
				{
					if (dilatedMask.at<uchar>(y, x) == 0)
					{
						int setPixels = 0;

						for (int yy = y - halfSize; yy <= y + halfSize; yy++)
						{
							if (yy < 0 || yy >= height)  continue;

							for (int xx = x - halfSize; xx <= x + halfSize; xx++)
							{
								if (xx<0 || xx >= width) continue;

								if (inputMask.at<uchar>(yy, xx) != 0) setPixels++;

							}
						}  //end kernel y - loop

						if (setPixels > minNrSetPixels) dilatedMask.at<uchar>(y, x) = (uchar)foregroundLevel;

					} //end mask == 0
				}
			}

			return dilatedMask;
		}

		//Removes isolated blobs in the background of maskIm, that are smaller than a given percentage of
		// the whole image size.The percentage is given as a number between 0 and 1.
		cv::Mat removeBlobs(cv::Mat maskIm, double percentage)
		{
			if (maskIm.channels() > 1)
			{
				cv::cvtColor(maskIm, maskIm, cv::COLOR_RGB2GRAY);
			}

			int imheight = maskIm.rows;
			int imwidth = maskIm.cols;

			//Assume that the border pixels are not part of the checkerboard and
			// therefore belong to the background
			//edgeMask([1 height], 1:width) = 0;
			//edgeMask(1:height, [1 width]) = 0;
			for (int y = 1; y < imheight - 1; y++)
			{
				maskIm.at<uchar>(y, 0) = maskIm.at<uchar>(y, imwidth - 1) = 0;
			}
			for (int x = 0; x < imwidth; x++)
			{
				maskIm.at<uchar>(0, x) = maskIm.at<uchar>(imheight - 1, x) = 0;
			}

			cv::Mat outMask;
			maskIm.copyTo(outMask);
			double smallThreshold = maskIm.total()*percentage;
			cv::floodFill(outMask, cv::Point(0, 0), 255);

			for (int y = 0; y < imheight; y++)
			for (int x = 0; x < imwidth; x++)
			{
				if (maskIm.at<uchar>(y, x) || !outMask.at<uchar>(y, x))
				{
					outMask.at<uchar>(y, x) = 255;
				}
				else
					outMask.at<uchar>(y, x) = 0;
			}

			//connected components
			cv::Mat labIm, stats, centroids;
			cv::connectedComponentsWithStats(outMask, labIm, stats, centroids);
			for (int y = 0; y < imheight; y++)
			for (int x = 0; x < imwidth; x++)
			{
				int m = labIm.at<int>(y, x);
				if (m > 0 && stats.at<int>(m, cv::CC_STAT_AREA) > smallThreshold)
				{
					outMask.at<uchar>(y, x) = maskIm.at<uchar>(y, x);
				}
				else
				{
					outMask.at<uchar>(y, x) = 0;
				}
			}

			return outMask;
		}

		//combines saddle points that are close
		//input: 
		//  edgeMask: input edge map
		//  saddleCandidates_: binary mask of saddle candidate points (output of findSaddlePointCandidates)
		//  saddleCombinationHalfSize: half size of the local window where saddle points are combined
		//  
		// output:
		//  combinedSaddleMask: binary mask of representative saddle points (one representative per combined group)
		//  saddleLabelImage: label image where original saddle points that are combined to the same group are given the same label
		void combineSaddles(cv::Mat edgeMask, cv::Mat saddleCandidates_, int saddleCombinationHalfSize, cv::Mat& combinedSaddleMask,
			cv::Mat& saddleLabelImage)
		{
			cv::Mat saddleCandidates;
			saddleCandidates_.copyTo(saddleCandidates);
			saddleCandidates_.copyTo(combinedSaddleMask);

			int imwidth = edgeMask.cols;
			int imheight = edgeMask.rows;

			//Remove saddles in margin regions in order to eliminate bounds checks in
			//region growing
			for (int x = 0; x < imwidth; x++)
			{
				for (int y = 0; y < saddleCombinationHalfSize; y++)
				{
					edgeMask.at<uchar>(y, x) = 0;
				}

				for (int y = imheight - saddleCombinationHalfSize; y < imheight; y++)
				{
					edgeMask.at<uchar>(y, x) = 0;
				}
			}

			for (int y = saddleCombinationHalfSize; y < imheight - saddleCombinationHalfSize; y++)
			{
				for (int x = 0; x < saddleCombinationHalfSize; x++)
				{
					edgeMask.at<uchar>(y, x) = 0;
				}

				for (int x = imwidth - saddleCombinationHalfSize; x < imwidth; x++)
				{
					edgeMask.at<uchar>(y, x) = 0;
				}
			}

			saddleLabelImage = cv::Mat::zeros(imheight, imwidth, CV_32S);

			cv::Mat visited = cv::Mat::zeros(imheight, imwidth, CV_32S);

			int segmentNumber = 1;
			int totalSaddleCount = 0;

			for (int y = 0; y < imheight; y++)
			for (int x = 0; x < imwidth; x++)
			{
				if (combinedSaddleMask.at<uchar>(y, x))
				{
					std::list<cv::Point> currentSaddleList;
					//csi = currentSaddleList.iterator;
					currentSaddleList.push_back(cv::Point(x, y));
					saddleLabelImage.at<int>(y, x) = segmentNumber;
					saddleCandidates.at<uchar>(y, x) = 0;

					while (!currentSaddleList.empty())
					{
						std::list<cv::Point> newSaddleList;

						for (auto & s : currentSaddleList)
						{
							int sX = s.x;
							int sY = s.y;
							totalSaddleCount++;

							std::list<cv::Point> currentPointNeighborhoodList;
							currentPointNeighborhoodList.push_back(cv::Point(sX, sY));

							visited.at<int>(sY, sX) = totalSaddleCount;

							for (int radiusCount = 1; radiusCount <= saddleCombinationHalfSize; radiusCount++)
							{
								std::list<cv::Point> newNeighborhoodList;

								for (auto & p : currentPointNeighborhoodList)
								{
									int pX = p.x;
									int pY = p.y;
									for (int j = -1; j <= 1; j++)
									for (int i = -1; i <= 1; i++)
									{
										int yj = pY + j;
										int xi = pX + i;
										if (edgeMask.at<uchar>(yj, xi) > 0 && visited.at<int>(yj, xi) != totalSaddleCount) // go only in unvisited points
										{
											visited.at<int>(yj, xi) = totalSaddleCount;
											newNeighborhoodList.push_back(cv::Point(xi, yj));
											if (combinedSaddleMask.at<uchar>(yj, xi) > 0)
											{
												combinedSaddleMask.at<uchar>(yj, xi) = 0;
												saddleLabelImage.at<int>(yj, xi) = segmentNumber;
												newSaddleList.push_back(cv::Point(xi, yj));
											}
										}
									}
								}
								currentPointNeighborhoodList = newNeighborhoodList;
							}
						}

						currentSaddleList = newSaddleList;
					}

					// restore the saddle
					combinedSaddleMask.at<uchar>(y, x) = 255;
					//Increment the saddle cluster label
					segmentNumber++;

				}
			}

		}

		bool testForCheckerboard(cv::Mat edgelabIm, int k, cv::Mat saddleLabelImage, cv::Mat combinedSaddleMask, int ncornerX, int ncornerY,
			std::vector<cv::Point2f>& initialcorners)
		{
			bool isCheckerboard = false;

			int imheight = saddleLabelImage.rows;
			int imwidth = saddleLabelImage.cols;

			cv::Mat saddle_locallabIm = cv::Mat::zeros(imheight, imwidth, CV_32S);
			cv::Mat edgeLocalMask = cv::Mat::zeros(imheight, imwidth, CV_8U);
			cv::Mat labmapMat = cv::Mat::zeros(imheight*imwidth, 1, CV_32S);
			int* labmap = (int*)labmapMat.data;

			int labcount = 0;
			for (int y = 0; y<imheight; y++)
			for (int x = 0; x < imwidth; x++)
			{
				if (edgelabIm.at<int>(y, x) == k)
				{
					edgeLocalMask.at<uchar>(y, x) = 255;

					if (combinedSaddleMask.at<uchar>(y, x) > 0)
					{
						labcount++;
						saddle_locallabIm.at<int>(y, x) = labcount;
						labmap[saddleLabelImage.at<int>(y, x)] = labcount;
					}
				}
			}

			int desiredcorners = ncornerX*ncornerY;
			desiredcorners = 24;
			int desiredtwoneighborcorners = 3;
			int desiredthreeneighborcorners = 12;
			int desiredfourneighborcorners = 9;

			if (labcount == desiredcorners)
			{
				std::cout << "found a checkerboard pattern\n";

				cv::Mat distim, pout_x, pout_y;
				voronoi(saddle_locallabIm, edgeLocalMask, distim, pout_x, pout_y, 10000);

				const int cx8[8] = { -1, -1, 0, 1, 1, 1, 0, -1 }, cy8[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
				cv::Mat adjacency = cv::Mat::zeros(desiredcorners + 1, desiredcorners + 1, CV_8U);

				//compute the adjacency between saddle points 
				for (int y = 1; y<imheight - 1; y++)
				for (int x = 1; x < imwidth - 1; x++)
				{
					if (saddle_locallabIm.at<int>(y, x) < 0)
					{
						//get the list of local adjacent labels
						std::vector<int> locallabs;
						for (int ct = 0; ct < 8; ct++)
						{
							int i = y + cy8[ct];
							int j = x + cx8[ct];
							if (saddle_locallabIm.at<int>(i, j) > 0)
							{
								locallabs.push_back(saddle_locallabIm.at<int>(i, j));
							}
						}

						//updates the adjacency matrix
						for (int u = 0; u < locallabs.size(); u++)
						for (int v = u + 1; v < locallabs.size(); v++)
						{
							int l1 = locallabs[u];
							int l2 = locallabs[v];
							if (l1 != l2)
							{
								adjacency.at<uchar>(l1, l2) = adjacency.at<uchar>(l2, l1) = 1;
							}
						}
					}
				}

				//compute neighbors for each saddle point
				int twoneighbors = 0;
				int threeneighbors = 0;
				int fourneighbors = 0;
				for (int i = 1; i <= desiredcorners; i++)
				{
					int neighbors = 0;
					for (int j = 1; j <= desiredcorners; j++)
					if (adjacency.at<uchar>(i, j) > 0) neighbors++;
					if (neighbors == 4)
					{
						fourneighbors++;
					}
					else if (neighbors == 3)
					{
						threeneighbors++;
					}
					else if (neighbors == 2)
					{
						twoneighbors++;
					}
				}


				if (twoneighbors == desiredtwoneighborcorners && threeneighbors == desiredthreeneighborcorners && fourneighbors == desiredfourneighborcorners)
				{
					isCheckerboard = true;

					//computes the corners
					initialcorners.resize(desiredcorners);
					cv::Mat saddlecnt = cv::Mat::zeros(desiredcorners, 1, CV_32S);
					for (int y = 0; y<imheight; y++)
					for (int x = 0; x < imwidth; x++)
					{
						int q = saddleLabelImage.at<int>(y, x);
						int m = labmap[q];
						if (m > 0)
						{
							saddlecnt.at<int>(m - 1, 0)++;
							initialcorners[m - 1].x += x;
							initialcorners[m - 1].y += y;
						}
					}

					for (int m = 0; m < desiredcorners; m++)
					{
						initialcorners[m].x /= saddlecnt.at<int>(m, 0);
						initialcorners[m].y /= saddlecnt.at<int>(m, 0);
					}
				}
			}

			return isCheckerboard;
		}

		//Input:
		//  removeNonloopMask: input edge map that contains an entire checkerboard
		//  saddleLabelImage: label image of saddle points (from output of combineSaddles)
		//  combinedSaddleMask: binary mask of representative saddle points (also from output of combineSaddles)
		//  ncornerX:  number of corner points in each row
		//  ncornerY:  number of corner points in each column
		//  downSampleFac: downsampling factor (from the output of RochadeResizing)
		//
		//Output:
		//  returns true if a checkerboard is detected, in which case the output array initialcorners will contain the coordinates
		//  of the corners (corrected for downsampling factor). 
		//  returns false if it fails detecting a checkerboard
		bool verifyCheckerboard(cv::Mat removeNonloopMask, cv::Mat saddleLabelImage, cv::Mat combinedSaddleMask, int ncornerX, int ncornerY,
			double downSampleFac, std::vector<cv::Point2f>& initialcorners)
		{
			cv::Mat edgelabIm, edgelabstats, edgelabcentroids;
			int edgenccomps = cv::connectedComponentsWithStats(removeNonloopMask, edgelabIm, edgelabstats, edgelabcentroids);

			for (int k = 1; k < edgenccomps; k++)
			{
				if (testForCheckerboard(edgelabIm, k, saddleLabelImage, combinedSaddleMask, ncornerX, ncornerY, initialcorners))
				{
					for (auto & p : initialcorners)
					{
						p.x *= (float) downSampleFac;
						p.y *= (float) downSampleFac;
					}
					return true;
				}
			}

			return false;
		}

		// refines checkerboard corners by quadratic fitting
		// input:
		//     imageInput: grayscale input image
		//     initialcorners: array of initial corner points
		//     halfSizePatch: the half size of the smoothing kernel and also the local neighborhood for quadratic fitting
		// output:
		//     refinedcorners: array of refined corner points
		//     refinementSuccessful: returned boolean values indicating refinement success/failure of each input corner
		void RochadeRefine(cv::Mat imageInput, std::vector<cv::Point2f> initialcorners, int halfSizePatch,
			std::vector<cv::Point2f>& refinedcorners, std::vector<bool>& refinementSuccessful)
		{
			// Compute kernel
			cv::Mat kernel = cv::Mat::zeros(halfSizePatch * 2 + 1, halfSizePatch * 2 + 1, CV_64F);
			int center = halfSizePatch;
			int maxVal = halfSizePatch + 1;

			double sk = 0;
			for (int n = 0; n < halfSizePatch * 2 + 1; n++)
			{
				for (int m = 0; m < halfSizePatch * 2 + 1; m++)
				{
					double t = maxVal - sqrt((center - m)*(center - m) + (center - n)*(center - n));
					kernel.at<double>(n, m) = t < 0 ? 0 : t;
					sk += kernel.at<double>(n, m);
				}
			}

			for (int n = 0; n < halfSizePatch * 2 + 1; n++)
			{
				for (int m = 0; m < halfSizePatch * 2 + 1; m++)
				{
					kernel.at<double>(n, m) /= sk;
				}
			}

			int height = imageInput.rows;
			int width = imageInput.cols;

			//polynomial fitting
			for (auto & pcorner : initialcorners)
			{
				double initialX = pcorner.x;
				double initialY = pcorner.y;

				if (initialY - halfSizePatch * 2 < 0 || initialY + halfSizePatch * 2 >= height || initialX - halfSizePatch * 2 < 0 || initialX + halfSizePatch * 2 >= width)
				{
					refinedcorners.push_back(pcorner);
					refinementSuccessful.push_back(0);
					continue;
				}


				//cropping a local patch
				cv::Mat patch = imageInput(cv::Rect((int)(initialX - halfSizePatch * 2), (int)(initialY - halfSizePatch * 2),
					4 * halfSizePatch + 1, 4 * halfSizePatch + 1));

				//filtering
				cv::Mat filteredPatch;
				cv::filter2D(patch, filteredPatch, CV_64F, kernel);
				//patch.convertTo(filteredPatch, CV_64F);

				//refcnt++;
				//cv::Mat filteredPatchInt;
				//filteredPatch.convertTo(filteredPatchInt, CV_8U);
				//std::ostringstream ss;
				//ss << "filteredPatch" << refcnt << ".png";
				//cv::imwrite(ss.str(), filteredPatchInt);

				//fit 2d polynomial
				//f = p[0]*x*x + p[1]*y*y + p[2]*x*y + p[3] * x + p[4] * y + p[5];
				cv::Mat b = cv::Mat::zeros(6, 1, CV_64F);
				cv::Mat A = cv::Mat::zeros(6, 6, CV_64F);
				std::vector<double> rt;
				rt.resize(6);

				for (int y = -halfSizePatch; y <= halfSizePatch; y++)
				for (int x = -halfSizePatch; x <= halfSizePatch; x++)
				{
					double f = filteredPatch.at<double>(y + halfSizePatch * 2, x + halfSizePatch * 2);

					rt[0] = x*x;
					rt[1] = y*y;
					rt[2] = x*y;
					rt[3] = x;
					rt[4] = y;
					rt[5] = 1;

					for (int i = 0; i < 6; i++)
					{
						for (int j = i; j < 6; j++)
						{
							A.at<double>(i, j) += rt[i] * rt[j];
						}
						b.at<double>(i) += rt[i] * f;
					}
				}

				for (int i = 0; i < 6; i++)
				for (int j = 0; j<i; j++)
				{
					A.at<double>(i, j) = A.at<double>(j, i);
				}

				cv::Mat p = A.inv(cv::DECOMP_CHOLESKY)*b;

				//double err = 0;
				//for (int y = -halfSizePatch; y <= halfSizePatch; y++)
				//	for (int x = -halfSizePatch; x <= halfSizePatch; x++)
				//	{
				//		double frec = p.at<double>(0) * x*x + p.at<double>(1) * y*y + p.at<double>(2) * x*y
				//			+ p.at<double>(3) * x + p.at<double>(4) * y + p.at<double>(5);
				//		double e = frec - filteredPatch.at<double>(y + halfSizePatch * 2, x + halfSizePatch * 2);
				//		err += e*e;
				//		filteredPatch.at<double>(y + halfSizePatch * 2, x + halfSizePatch * 2)
				//			= frec;
				//	}
				//std::cout << p << std::endl;
				//std::cout << "err=" << err << std::endl;
				//filteredPatch.convertTo(filteredPatchInt, CV_8U);
				//ss.str("");
				//ss << "filteredPatchRec" << refcnt << ".png";
				//cv::imwrite(ss.str(), filteredPatchInt);

				double fxx = 2 * p.at<double>(0);
				double fyy = 2 * p.at<double>(1);
				double fxy = p.at<double>(2);
				double fx = p.at<double>(3);
				double fy = p.at<double>(4);

				double hessDet = fxx*fyy - fxy*fxy;

				bool saddleDetected = 0;

				if (fxy == 0)
				{
					if (fxx == 0)
					{
						// not a saddle point
					}
					else if (fyy == 0)
					{
						// not a saddle point
					}
					else
					{
						if (hessDet > 0)
						{
							// not a saddle point
						}
						else
						{
							saddleDetected = 1;
						}
					}
				}
				else
				{
					if (hessDet <= 0)
					{
						saddleDetected = 1;
					}
				}

				if (saddleDetected)
				{
					//fxx*x + fxy*y + fx = 0;
					//fxy*x + fyy*y + fy = 0;
					double xx = -(fyy*fx - fxy*fy) / hessDet;
					double yy = -(fxx*fy - fxy*fx) / hessDet;

					refinedcorners.push_back(cv::Point2f((float)(initialX + xx), (float)(initialY + yy)));
					refinementSuccessful.push_back(1);
				}
				else
				{
					refinedcorners.push_back(pcorner);
					refinementSuccessful.push_back(0);
				}
			}

		}

		void RochadeCornerVisualize(cv::Mat im, std::vector<cv::Point2f> corners, cv::Mat& imcorners)
		{
			if (im.channels() == 1)
			{
				cv::cvtColor(im, imcorners, CV_GRAY2RGB);
			}
			else
			{
				im.copyTo(imcorners);
			}

			int msize = 20;
			if (msize < im.cols / 100)
			{
				msize = im.cols / 100;
			}

			int thick = im.cols / 1000;
			if (thick < 1) thick = 1;

			for (int i = 0; i < corners.size(); i++)
			{
				cv::drawMarker(imcorners, corners[i], cv::Scalar(0, 255, 0), 0, msize);

				cv::circle(imcorners, corners[i], (int)(0.5*msize), cv::Scalar(0, 0, 255), thick);
			}
		}

		int main()
		{
			cv::Mat im = cv::imread("C:/Hieu/project2017/TOFRGBCalibration/Nikon_H.1.png");
			if (im.channels() == 3)
			{
				cv::cvtColor(im, im, CV_BGR2GRAY);
			}

			double localRelativeThreshold = 0.4;
			double maximumHoleSizeInPercent = 0.01 / 100;

			double downSamplingThreshold = 1000;

			double upSampleFac = 1;
			double downSampleFac = 1;

			//resizing
			cv::Mat imageInput = RochadeResizing(im, downSamplingThreshold, upSampleFac, downSampleFac);

			int imheight = imageInput.rows;
			int imwidth = imageInput.cols;

			//////////////
			cv::Mat edgeImage = Scharr(imageInput);

			cv::Mat outim;
			edgeImage.convertTo(outim, CV_8U);
			cv::imwrite("scharr.png", outim);

			//edgeImage = imclose(edgeImage, strel('rectangle', [3 3])); %optional
			cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
			cv::morphologyEx(edgeImage, edgeImage, cv::MORPH_CLOSE, kernel3);

			cv::imwrite("edgeclosed.png", edgeImage);

			cv::Mat edgeMask = RochadeLocalThresholding(edgeImage, localRelativeThreshold);

			cv::imwrite("edgeMask.png", edgeMask);

			//edgeMask = conditionalDilation(edgeMask, 1, 5);
			cv::Mat edgeMaskDil = fillHolesAndNotches(edgeMask, 1, 5);
			for (int y = 0; y < imheight; y++)
			for (int x = 0; x < imwidth; x++)
			{
				outim.at<uchar>(y, x) = edgeMaskDil.at<uchar>(y, x) > 0 ? 255 : 0;
			}
			edgeMask = edgeMaskDil;
			cv::imwrite("edgeMaskDilate.png", outim);

			//cv::Mat wrench;
			//cv::cvtColor(cv::imread("C:/Hieu/Prog/QT/VideoPIONEER/Release/wrench3seg.tif"), wrench, cv::COLOR_RGB2GRAY);
			//cv::Mat wrenchDil = fillNotches(wrench, 2, 5);
			//cv::imwrite("wrenchdil.png", wrenchDil);
			//cv::Mat wrenchfill = removeBlobs(wrench, 0.001);
			//cv::imwrite("wrenchfill.png", wrenchfill);

			//return 0;
			// Fill small holes which are part of the background
			edgeMask = removeBlobs(edgeMask, maximumHoleSizeInPercent);	//optional
			for (int y = 0; y < imheight; y++)
			for (int x = 0; x < imwidth; x++)
			{
				outim.at<uchar>(y, x) = edgeMask.at<uchar>(y, x) > 0 ? 255 : 0;
			}

			cv::imwrite("edgeMaskfillholes.png", outim);

			//centerlinesThick = bwmorph(edgeMask, 'skel'); %Thick centerlines, first part of centline extraction
			double CurvatureKernelSize = 15;
			double MinCurvatureStrength = 0.1;
			cv::Mat skel_im;
			Voronoi_skeleton(edgeMask, CurvatureKernelSize, MinCurvatureStrength, skel_im);
			cv::imwrite("thinEdgeMask.png", skel_im);

			cv::Mat removeNonloopMask = removeNonloop(skel_im);

			cv::imwrite("removeNonloopMask.png", removeNonloopMask);

			cv::Mat saddleCandidates = findSaddlePointCandidates(removeNonloopMask);

			cv::imwrite("saddleCandidates.png", saddleCandidates);

			int saddleCombinationHalfSize = 2 + imwidth / 2000;

			cv::Mat combinedSaddleMask, saddleLabelImage;
			combineSaddles(removeNonloopMask, saddleCandidates, saddleCombinationHalfSize, combinedSaddleMask, saddleLabelImage);

			cv::imwrite("combinedSaddleMask.png", combinedSaddleMask);
			cv::imwrite("saddleLabelImage.png", saddleLabelImage);

			//test for checkerboard
			std::vector<cv::Point2f>initialcorners;
			int ncornerX = 5, ncornerY = 5;		//number of intersection points in each row and column respectively

			int refinementKernelHalfSize = 20;

			if (verifyCheckerboard(removeNonloopMask, saddleLabelImage, combinedSaddleMask, ncornerX, ncornerY, downSampleFac, initialcorners))
			{
				cv::Mat imcorners;
				RochadeCornerVisualize(im, initialcorners, imcorners);
				cv::imwrite("imcorners.png", imcorners);

				std::vector<cv::Point2f> refinedcorners;
				std::vector<bool> refinementSuccessful;

				RochadeRefine(im, initialcorners, refinementKernelHalfSize, refinedcorners, refinementSuccessful);

				RochadeCornerVisualize(im, refinedcorners, imcorners);
				cv::imwrite("imcorners_refined.png", imcorners);
			}

			return 0;
		}
	}
}

