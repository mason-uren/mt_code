#include "voronoi.h"

/* labim is a label image
* vornoi diagram is specified by the returned value -1 in labim
* pout_x, pout_y: output nearest points
*/
namespace BoeingMetrology
{
	namespace ImageProcessing
	{
		void voronoi_nomask(cv::Mat& labim, cv::Mat& distim, cv::Mat& pout_x, cv::Mat& pout_y, int qmax)
		{
			/* Chamfer neighborhood*/
			int xn[16] = { 1, 1, -1, -1, 2, 0, -2, -1, 1, 2, 1, 0, -1, -2, -1, 1 };
			int yn[16] = { -1, 1, 1, -2, 0, 2, 0, -2, -1, 0, 1, 2, 1, 0, -1, -1 };
			/* distance values*/
			int s[16] = { 5, 5, 5, 5, 7, 7, 7, 7, 11, 11, 11, 11, 11, 11, 11, 11 };
			int ctn = 16;

			int imheight = labim.rows;
			int imwidth = labim.cols;

			distim = cv::Mat(imheight, imwidth, CV_32S);
			pout_x = cv::Mat(imheight, imwidth, CV_32S);
			pout_y = cv::Mat(imheight, imwidth, CV_32S);

			//const int cx8[8] = {-1,-1, 0, 1,1,1,0,-1},  cy8[8] = { 0,-1,-1,-1,0,1,1, 1};
			const int cx4[4] = { -1, 0, 1, 0 },
				cy4[4] = { 0, -1, 0, 1 };

			std::list<int> *blist;
			blist = new std::list<int>[qmax + 1];
			const int imsize = imwidth*imheight;
			std::list<int>::iterator *iter = new std::list<int>::iterator[imsize];
			int *process = new int[imsize];

			for (int y = 0, npix = 0; y < imheight; y++) for (int x = 0; x < imwidth; x++, npix++)
			{
				process[npix] = 0;
				int d = labim.at<int>(y, x) > 0 ? 0 : qmax;
				distim.at<int>(y, x) = d;
				pout_x.at<int>(y, x) = x;
				pout_y.at<int>(y, x) = y;
				blist[d].push_back(npix);	/* put all points on queues with distance beging priority*/
				iter[npix] = blist[d].end();
				iter[npix]--;
			}

            long npix, qpix;
			int qmin = 0;
			int done = 0;
			while (!done)
			{
				if (blist[qmin].empty())
				{
					// determine the new qmin
					while (qmin < qmax && blist[qmin].empty()) qmin++;
					done = blist[qmin].empty();
				}
				else
				{
					npix = blist[qmin].front();
					int x = npix % imwidth;
					int y = npix / imwidth;

					blist[qmin].pop_front();
					int i, j, ct;
					/* update the distance function in the 5x5 neighborhood	*/
					for (i = x - 1, j = y, ct = 0; ct < ctn; i += xn[ct], j += yn[ct], ct++)
						if (i >= 0 && i < imwidth && j >= 0 && j < imheight)
						{
							qpix = i + imwidth*j;
							int oldq = distim.at<int>(j, i);
							int cn = (int)(distim.at<int>(y, x) + s[ct]);
							int change_q;
							if (cn < distim.at<int>(j, i))
							{
								distim.at<int>(j, i) = cn;	/* update distance	*/

								pout_x.at<int>(j, i) = pout_x.at<int>(y, x);	/* nearest point	*/
								pout_y.at<int>(j, i) = pout_y.at<int>(y, x);
								change_q = 1;
							}
							else change_q = 0;

							/* if priority of a neighboring point changes and if the point is still on the queues
							then move it to a new queue with higher priority
							*/
							if (!process[qpix])
							{
								if (change_q)
								{
									blist[oldq].erase(iter[qpix]);
									blist[cn].push_back(qpix);
									iter[qpix] = blist[cn].end();
									iter[qpix]--;
									if (cn < qmin) qmin = cn;
								}
							}
						}

					if (labim.at<int>(y, x) == 0)
					{
						/* set the label to the label of the nearest point*/
						int lab = labim.at<int>(pout_y.at<int>(y, x), pout_x.at<int>(y, x));
						labim.at<int>(y, x) = lab;

						/* if any point in the 4-connectivity neighborhood comes from other influence zones
						then set pB = -1. The Voronoi diagram will be 8-connected */
						for (ct = 0; ct < 4; ct++)
						{
							i = x + cx4[ct];
							j = y + cy4[ct];
							if (i >= 0 && i<imwidth && j >= 0 && j<imheight)
							{
								int labneighbor = labim.at<int>(j, i);
								if (labneighbor <0) labneighbor = -labneighbor;

								if (labneighbor != lab &&  labneighbor>0/*&& process[i+imwidth*j]==1*/)
								{
									labim.at<int>(y, x) = -lab;
									break;
								}
							}
						}
					}

					/*mark the point as processed*/
					process[npix] = 1;
				}
			}

			delete[] blist;
			delete[] process;
			delete[] iter;

		}

		/*
		* vornoi diagram using input mask image (not used if mask==NULL)
		* labim is a label image
		* vornoi diagram is specified by the returned value -1 in pB
		*/
		void voronoi(cv::Mat& labim, cv::Mat mask,
			cv::Mat& distim, cv::Mat& pout_x, cv::Mat& pout_y, int qmax)
		{
			if (mask.empty())
			{
				voronoi_nomask(labim, distim, pout_x, pout_y, qmax);
				return;
			}

			int imheight = labim.rows;
			int imwidth = labim.cols;

			distim = cv::Mat(imheight, imwidth, CV_32S);
			pout_x = cv::Mat(imheight, imwidth, CV_32S);
			pout_y = cv::Mat(imheight, imwidth, CV_32S);

			/* Chamfer neighborhood*/
			int xn[16] = { 1, 1, -1, -1, 2, 0, -2, -1, 1, 2, 1, 0, -1, -2, -1, 1 };
			int yn[16] = { -1, 1, 1, -2, 0, 2, 0, -2, -1, 0, 1, 2, 1, 0, -1, -1 };
			/* distance values*/
			int s[16] = { 5, 5, 5, 5, 7, 7, 7, 7, 11, 11, 11, 11, 11, 11, 11, 11 };
			int ctn = 16;


			//const int cx8[8] = {-1,-1, 0, 1,1,1,0,-1},  cy8[8] = { 0,-1,-1,-1,0,1,1, 1};
			const int cx4[4] = { -1, 0, 1, 0 },
				cy4[4] = { 0, -1, 0, 1 };

			std::list<int> *blist;
			blist = new std::list<int>[qmax + 1];
			const int imsize = imwidth*imheight;
			std::list<int>::iterator *iter = new std::list<int>::iterator[imsize];
			int *process = new int[imsize];

			for (int y = 0, npix = 0; y < imheight; y++) for (int x = 0; x < imwidth; x++, npix++)
				if (mask.at<uchar>(y, x)>0)
				{
					process[npix] = 0;
					int d = (labim.at<int>(y, x) > 0) ? 0 : qmax;
					distim.at<int>(y, x) = d;
					pout_x.at<int>(y, x) = x;
					pout_y.at<int>(y, x) = y;
					blist[d].push_back(npix);	/* put all points on queues with distance being the priority*/
					iter[npix] = blist[d].end();
					iter[npix]--;
				}

			int qmin = 0;
			int done = 0;
            long npix, qpix;
			while (!done)
			{
				if (blist[qmin].empty())
				{
					// determine the new qmin
					while (qmin < qmax && blist[qmin].empty()) qmin++;
					done = blist[qmin].empty();
				}
				else
				{
					npix = blist[qmin].front();
					int x = npix % imwidth;
					int y = npix / imwidth;

					blist[qmin].pop_front();
					int i, j, ct;
					/* update the distance function in the 5x5 neighborhood	*/
					for (i = x - 1, j = y, ct = 0; ct < ctn; i += xn[ct], j += yn[ct], ct++)
						if (i >= 0 && i < imwidth && j >= 0 && j < imheight)
							if (mask.at<uchar>(j, i)>0)
							{
								qpix = i + imwidth*j;
								int oldq = distim.at<int>(j, i);
								int cn = distim.at<int>(y, x) + s[ct];

								int change_q;
								if (cn < oldq)
								{
									distim.at<int>(j, i) = cn;	/* update distance	*/

									pout_x.at<int>(j, i) = pout_x.at<int>(y, x);	/* passing on info of nearest point	*/
									pout_y.at<int>(j, i) = pout_y.at<int>(y, x);
									change_q = 1;
								}
								else change_q = 0;

								/* if priority of a neighboring point changes and if the point is still on the queues
								then move it to a new queue with higher priority
								*/
								if (!process[qpix])
								{
									if (change_q)
									{
										blist[oldq].erase(iter[qpix]);
										blist[cn].push_back(qpix);
										iter[qpix] = blist[cn].end();
										iter[qpix]--;
										if (cn < qmin) qmin = cn;
									}
								}
							}

					if (labim.at<int>(y, x) == 0)
					{
						/* set the label to the label of the nearest point*/
						int lab = labim.at<int>(pout_y.at<int>(y, x), pout_x.at<int>(y, x));
						labim.at<int>(y, x) = lab;

						/* if any point in the 4-connectivity neighborhood comes from other influence zones
						then set labim = -1. The Voronoi diagram will be 8-connected */
						for (ct = 0; ct < 4; ct++)
						{
							i = x + cx4[ct];
							j = y + cy4[ct];
							if (i >= 0 && i<imwidth && j >= 0 && j<imheight && mask.at<uchar>(j, i)>0)
							{
								int labneighbor = labim.at<int>(j, i);
								if (labneighbor <0) labneighbor = -labneighbor;

								if (labneighbor != lab &&  labneighbor>0 )
								{
									labim.at<int>(y, x) = -lab;
									break;
								}
							}
						}
					}

					/*mark the point as processed*/
					process[npix] = 1;
				}
			}

			delete[] blist;
			delete[] process;
			delete[] iter;

		}


		void voronoi(cv::Mat& labim, cv::Mat mask, cv::Mat& distim, int qmax)
		{
			cv::Mat poutx, pouty;

			voronoi(labim, mask, distim, poutx, pouty, qmax);
		}
	}
}
