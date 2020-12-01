#include "skeletonization.h"
#include "voronoi.h"

void Voronoi_skeleton_fromLabel(cv::Mat labim0, const int nobj, double sigc, double curvthr,
	std::vector<std::list<cv::Point>> *endpoint, cv::Mat& skel_im, cv::Mat& distim);

void outer_contour_plateau(cv::Mat im, int level,
	int xmin, int ymin, int xmax, int ymax, std::vector<std::list<std::pair<int, int>>> &pout);

void listpair2array(std::list<std::pair<int, int>> &pout, int* xcon, int *ycon);

void contour_curvature(int *xc, int*yc, const int n, double *cout, double sig);

///////////////////Skeletonization////////////////////////////////////
/*
* Computes the skeleton of a binary image  using the Voronoi diagram of
* contour segments split at points of maximal curvature.
*
* Input:
* inMask: input binary image
* CurvatureKernelSize: gaussian kernel for curvature calculation (15.0)
* MinCurvatureStrength:  threshold for curvature maxima (0.1)
*
* output:
* skelMask: binary image of the skeleton
*/
void BoeingMetrology::ImageProcessing::Voronoi_skeleton(cv::Mat inMask, double CurvatureKernelSize, double MinCurvatureStrength,
	cv::Mat& skelMask)
{
	if (inMask.channels()>1)
	{
		cv::cvtColor(inMask, inMask, cv::COLOR_RGB2GRAY);
	}
	cv::Mat labIm;
	int nccomps = cv::connectedComponents(inMask, labIm);

	cv::Mat distim;
	Voronoi_skeleton_fromLabel(labIm, nccomps - 1, CurvatureKernelSize, MinCurvatureStrength,
		NULL, skelMask, distim);
}


/*
* The skeleton of an object is determined as the Voronoi diagram of
* contour segments split at points of maximal curvature.
*
* labim0: label image with nobj objects
* sigc: gaussian kernel for curvature calculation (15.0)
* curvthr:  threshold for curvature maxima (0.1)
*
* output:
* endpoint: returned list of end points
* skelMask: binary image of the skeleton
* distim: 0: background points, positive values: distance to the nearest contour point
* boxes: list of bounding box parameters for each object
*/
void Voronoi_skeleton_fromLabel(cv::Mat labim0, const int nobj, double sigc, double curvthr,
	std::vector<std::vector<int>>& boxes, std::vector<std::list<cv::Point>> *endpoint,
	cv::Mat& skelMask, cv::Mat& distim)
{
	const int cx8[8] = { -1, -1, 0, 1, 1, 1, 0, -1 }, cy8[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };

	int height = labim0.rows;
	int width = labim0.cols;

	cv::Mat labim;
	labim0.copyTo(labim);

	/* set border to zero because later we need to write to object outer contour*/
	for (int y = 1; y < height - 1; y++)
	{
		labim.at<int>(y, 0) = labim.at<int>(y, width - 1) = 0;
	}
	for (int x = 0; x < width; x++)
	{
		labim.at<int>(0, x) = labim.at<int>(height - 1, x) = 0;
	}

	/* mask the region of interest where the voronoi diagram is computed*/
	cv::Mat mask(height, width, CV_8U);
	for (int y = 0; y<height; y++) for (int x = 0; x<width; x++)
	{
		mask.at<uchar>(y, x) = labim.at<int>(y, x)>0 ? 255 : 0;
	}

	boxes.resize(nobj + 1);
	/* find a bounding box for each object*/
	for (int i = 1; i <= nobj; i++)
	{
		boxes[i].resize(5);
		boxes[i][0] = width;   //xmin
		boxes[i][1] = height;   //ymin
		boxes[i][2] = -1;       //xmax
		boxes[i][3] = -1;       //ymax
		boxes[i][4] = 0;		//area
	}
	int l;
	for (int y = 0; y<height; y++) for (int x = 0; x<width; x++)
	{
		if ((l = labim.at<int>(y, x))>0)
		{
			if (boxes[l][0]>x) boxes[l][0] = x;
			if (boxes[l][1]>y) boxes[l][1] = y;
			if (boxes[l][2]<x) boxes[l][2] = x;
			if (boxes[l][3]<y) boxes[l][3] = y;
			boxes[l][4]++;
		}
	}

	/* initial setting for skeleton image*/
	distim = cv::Mat::zeros(height, width, CV_32S);
	cv::Mat skel_im = cv::Mat::zeros(height, width, CV_32S);

	/* and split the object contours at their local curvature maxima */
	int nl = 0;
	for (int m = 1; m <= nobj; m++)
	{
		if (boxes[m][4] == 0) continue;

		/* trace the OUTER contour of each object (caution:it may go outside image)*/
		std::vector<std::list<std::pair<int, int>>> pout;		//list of contour points
		outer_contour_plateau(labim, m, boxes[m][0], boxes[m][1], boxes[m][2], boxes[m][3], pout);

		int nloop = (int)pout.size();				//number of contour loops (including possible holes) in a binary object

		for (int k = 0; k< nloop; k++)
		{
			nl++;									//label for each contour segment
			/* include the boundary pixels to the region of interest*/
			std::list<std::pair<int, int>>::iterator pit = pout[k].begin();
			for (; pit != pout[k].end(); pit++) mask.at<uchar>(pit->second, pit->first) = 255;

			int n = (int)pout[k].size();
			int *xc = new int[n];
			int *yc = new int[n];
			double *curv = new double[n];
			listpair2array(pout[k], xc, yc);

			//calculating curvature
			contour_curvature(xc, yc, n, curv, sigc);

			int nlold = nl;
			int lastcorner = -1;
			for (int i = 0; i<n; i++)
			{
				skel_im.at<int>(yc[i], xc[i]) = nl;
				//check for high curvature corners
				if (curv[i]>curvthr)
				{
					//getting curvature of the preceding and the following pixels along the contour
					int nb = i - 1;
					if (nb<0) nb = n - 1;
					int na = i + 1;
					if (na >= n) na = 0;

					//check for local maxima
					if (curv[i]>curv[na] && curv[i] >curv[nb])
					{
						lastcorner = i;
						nl++;
					}
				}
			}

			if (lastcorner>0)
			{
				//the first and last contour segments will have the same label
				//so the label of last segment needs be corrected
				for (int i = n - 1; i>lastcorner; i--) skel_im.at<int>(yc[i], xc[i]) = nlold;
				nl--;
			}

			//printf("m=%i\n",m);

			delete[] xc;
			delete[] yc;
			delete[] curv;
		}

	}

	/* compute the Voronoi diagram of boundary segments, which will be the skeleton */
	int qmax = 10000;
	BoeingMetrology::ImageProcessing::voronoi(skel_im, mask, distim, qmax);

	if (endpoint != NULL)
	{
		endpoint->resize(nobj + 1);

		/* extract end points*/
		for (int m = 1; m <= nobj; m++)
		{
			for (int y = boxes[m][1] - 1; y <= boxes[m][3] + 1; y++)
			for (int x = boxes[m][0] - 1; x <= boxes[m][2] + 1; x++)
			{
				if (y >= 0 && y<height && x >= 0 && x<width)
				if (labim.at<int>(y, x) == m && skel_im.at<int>(y, x) < 0)
				{
					int cnt = 0;
					for (int ct = 0; ct<8; ct++)
					{
						int i = y + cy8[ct];
						int j = x + cx8[ct];
						if (i >= 0 && i<height && j >= 0 && j<width)
						if (skel_im.at<int>(i, j) < 0 && labim.at<int>(i, j) == m) cnt++;
					}
					if (cnt == 1)
					{
						endpoint->at(m).push_back(cv::Point(x, y));
					}
				}
			}
		}
	}

	skelMask = cv::Mat(height, width, CV_8U);
	//the Voronoi diagram is marked by the value -1
	for (int y = 0; y<height; y++) for (int x = 0; x<width; x++)
		skelMask.at<uchar>(y, x) = skel_im.at<int>(y, x) <0 ? 255 : 0;

}

void Voronoi_skeleton_fromLabel(cv::Mat labim0, const int nobj, double sigc, double curvthr,
	std::vector<std::list<cv::Point>> *endpoint, cv::Mat& skel_im, cv::Mat& distim)
{
	std::vector<std::vector<int>> boxes;

	Voronoi_skeleton_fromLabel(labim0, nobj, sigc, curvthr, boxes, endpoint, skel_im, distim);

}


//////////////Contour Tracing//////////////////////////////////
static const int cx8[8] = { -1,-1, 0, 1,1,1,0,-1 },
cy8[8] = { 0,-1,-1,-1,0,1,1, 1 };

static const int xnright8[8] = { 0,1,1,0,0,-1,-1, 0 },
ynright8[8] = { -1,0,0,1,1, 0, 0,-1 };

static const int xnleft8[8] = { 0,0,-1,-1, 0, 0,1,1 },
ynleft8[8] = { 1,1, 0, 0,-1,-1,0,0 };

static const int st_ch_right[8] = { 6,0,0,2,2,4,4,6 };
static const int  st_ch_left[8] = { 2,2,4,4,6,6,0,0 };

#define BAD_INITIAL_TRACE_POINT 11
#define HOLE_FOUND 12
#define LOOP_FOUND 13
#define NOOBJECTFOUND 14

#define TRACE_RIGHT 0
#define TRACE_LEFT  1

#define UNTIL_N_POINTS 0
#define EXIT_IF_LOOP 1

#define INNER_CONTOUR 0
#define OUTER_CONTOUR 1

template<class T>
int outer_contour_plateau_tmplt(cv::Mat im,
	int x0, int y0, int st0, int direction, const int npoints,
	int mode, std::list<std::pair<int, int>> &pout)
{
	int Height = im.rows;
	int Width = im.cols;

	//check whether the starting point is right or not
	if (x0 <0 || x0 >= Width || y0 < 0 || y0 >= Height) return BAD_INITIAL_TRACE_POINT;
	int lev = im.at<T>(y0, x0);

	int st = st0;
	int x = x0 + cx8[st];	/* (x,y) is a background point	*/
	int y = y0 + cy8[st];
	int bx0 = x; int by0 = y;
	int xyvalid = 1;
	if (x >= 0 && x<Width && y >= 0 && y< Height)
		if (im.at<T>(y, x) == lev) xyvalid = 0;					//return if the starting background point has intensity >= th
	if (!xyvalid) return BAD_INITIAL_TRACE_POINT;

	const int *xn8, *yn8, *st_ch;
	if (direction == TRACE_RIGHT)
	{
		xn8 = xnright8; yn8 = ynright8;
		st_ch = st_ch_right;
	}
	else
	{
		xn8 = xnleft8; yn8 = ynleft8;
		st_ch = st_ch_left;
	}

	int count = 0;
	pout.push_back(std::pair<int, int>(x, y));
	count++;
	int object_count = 0;
	int loop_found = 0;

	while (count < npoints)
	{
		int tx = x + xn8[st]; int ty = y + yn8[st];
		if (tx >= 0 && tx < Width && ty >= 0 && ty < Height &&
			im.at<T>(ty, tx) == lev)/* if an object point encountered	*/
		{
			st = st_ch[st];
			object_count++;
			if (object_count >= 4) break;
		}
		else
		{
			x = tx; y = ty;
			if (x == bx0 && y == by0)
			{
				loop_found = 1;
				if (mode == EXIT_IF_LOOP) break;
			}
			pout.push_back(std::pair<int, int>(x, y));
			count++;
			if (direction == TRACE_RIGHT)
			{
				if (++st == 8) st = 0;
			}
			else
			{
				if (--st == -1) st = 7;
			}
			object_count = 0;
		}
	}

	if (object_count >= 4) return HOLE_FOUND;
	else if (loop_found)	return LOOP_FOUND;
	else return 0;
}



/* trace all possible contours inclunding holes*/
template <class T>
void outer_contour_plateau_tmplt(cv::Mat im, int level,
	int xmin, int ymin, int xmax, int ymax, std::vector<std::list<std::pair<int, int>>> &pout)
{
	int imheight = im.rows;
	int imwidth = im.cols;

	cv::Mat mask = cv::Mat::zeros(ymax - ymin + 3, xmax - xmin + 3, CV_8U);

	const int nmaxpoints = imwidth*imheight;

	/* finding a starting point*/
	int st0 = 0;
	for (int y = ymin; y <= ymax; y++) for (int x = xmin; x <= xmax; x++)
		if (im.at<T>(y, x) == level)
		{
			int no_background_found = 1;
			for (int i = 0; i<8; i++)
			{
				int yy = y + cy8[i];
				int xx = x + cx8[i];
				if (yy >= 0 && yy <imheight && xx >= 0 && xx <imwidth && mask.at<uchar>(yy - ymin + 1, xx - xmin + 1) == 0
					&& im.at<T>(yy, xx) != level)
				{
					st0 = i;
					no_background_found = 0;

					break;
				}
			}

			if (!no_background_found)
			{
				std::list<std::pair<int, int>> cntr;
				pout.push_back(cntr);
				int n = (int)pout.size() - 1;

				outer_contour_plateau_tmplt<T>(im, x, y, st0, TRACE_RIGHT, nmaxpoints,
					EXIT_IF_LOOP, pout[n]);

				std::list<std::pair<int, int>>::iterator ptr = pout[n].begin();
				for (; ptr != pout[n].end(); ptr++)
				{
					int xx = ptr->first - xmin + 1;
					int yy = ptr->second - ymin + 1;
					if (yy >= 0 && yy <mask.rows && xx >= 0 && xx <mask.cols) mask.at<uchar>(yy, xx) = 1;

				}
			}
		}

	//free_imatrix(mask, ymin-1, ymax+1, xmin-1, xmax+1);
}

void outer_contour_plateau(cv::Mat im, int level,
	int xmin, int ymin, int xmax, int ymax, std::vector<std::list<std::pair<int, int>>> &pout)
{
	switch (im.type())
	{
	case CV_8U:
		outer_contour_plateau_tmplt<uchar>(im, level, xmin, ymin, xmax, ymax, pout);
		break;
	case CV_16U:
		outer_contour_plateau_tmplt<ushort>(im, level, xmin, ymin, xmax, ymax, pout);
		break;
	case CV_16S:
		outer_contour_plateau_tmplt<short>(im, level, xmin, ymin, xmax, ymax, pout);
		break;
	case CV_32S:
		outer_contour_plateau_tmplt<int>(im, level, xmin, ymin, xmax, ymax, pout);
		break;
	}
}

//convert from list<pair<>> to int*
void listpair2array(std::list<std::pair<int, int>> &pout, int* xcon, int *ycon)
{
	int i = 0;
	std::list<std::pair<int, int>>::iterator pp = pout.begin();
	for (; pp != pout.end(); pp++, i++)
	{
		xcon[i] = pp->first;
		ycon[i] = pp->second;
	}
}

//////////////////////////////contour curvature computation//////////////////

#define GLEN_MAX 70
#define FLEN_MAX 2*GLEN_MAX+1

//circular 1D convolution: x[0..n-1] * h[-hwidth2,hwidth2] = dx[0..n-1]
void cconvol11intdouble(int *x, const int n, double *h, int hwidth2, double *dx)
{
	if (n> hwidth2 + hwidth2)
	{
		for (int i = hwidth2; i<n - hwidth2; i++)
		{
			double s = 0;
			for (int k = -hwidth2; k <= hwidth2; k++) s += x[i - k] * h[k];
			dx[i] = s;
		}

		for (int i = 0; i<hwidth2; i++)
		{
			double s = 0;
			for (int k = -hwidth2; k <= i; k++) s += x[i - k] * h[k];
			for (int k = i + 1; k <= hwidth2; k++) s += x[n + i - k] * h[k];
			dx[i] = s;
		}

		for (int i = n - hwidth2; i<n; i++)
		{
			double s = 0;
			for (int k = -hwidth2; k<-(n - 1 - i); k++) s += x[i - k - n] * h[k];
			for (int k = -(n - 1 - i); k <= hwidth2; k++) s += x[i - k] * h[k];
			dx[i] = s;
		}
	}
	else
	{
		for (int i = 0; i<n; i++)
		{
			double s = 0;
			for (int k = -hwidth2; k <= hwidth2; k++)
			{
				int m = i - k;
				if (m >= n) m = m % n;
				if (m<0)
				{
					m = -((-m) % n);
					if (m<0) m += n;
				}
				s += x[m] * h[k];
			}
			dx[i] = s;
		}
	}
}

/* compute the gaussian derivative kernels*/
int contour_gauss_dkernel(double sigma, double *a1, double *a2)
{
	/* first compute the derivative kernel of size 0.5*sigma */
	double half_sigma = sigma / 2;
	double half_sigma2 = half_sigma*half_sigma;
	double half_aa[FLEN_MAX];
	double *const half_a = &half_aa[GLEN_MAX];
	half_a[0] = 0;
	double b1 = 1.0 / (half_sigma*sqrt(2 * CV_PI));
	double b2 = 0.5 / half_sigma2;
	int half_glen;
	for (int k = 1; ; k++)
	{
		double gf = exp(-k*k*b2)*b1;
		half_a[k] = -k*gf / half_sigma2;	//first order derivative
		half_a[-k] = -half_a[k];
		if (gf<0.01) { half_glen = k; break; }
	}
	/* the kernel for the second derivatives is obtained by convolution
	of two first derivative kernels of size 0.5*sigma*/
	int glen = 2 * half_glen;
	if (glen>GLEN_MAX)
	{
		std::cout << "glen=" << glen << " exceeds the limit!\n";
		return -1;
	}

	for (int k = -glen; k <= glen; k++)
	{
		a2[k] = 0;
		for (int i = -half_glen; i <= half_glen; i++)
			if (k - i >= -half_glen && k - i <= half_glen)
				a2[k] += half_a[k - i] * half_a[i];
	}

	/* now compute the first derivative kernel of size sigma*/
	double sigma2 = sigma*sigma;
	b1 = 1.0 / (sigma*sqrt(2 * 3.14159));
	b2 = 0.5 / sigma2;
	a1[0] = 0;
	for (int k = 1; k <= glen; k++)
	{
		double gf = exp(-k*k*b2)*b1;
		a1[k] = -k*gf / sigma2;	//first order derivative
		a1[-k] = -a1[k];
	}

	return glen;
}

void contour_derivative(int *xout, int *yout, const int n, double *dxt, double *dyt,
	double *a1, int glen)
{
	cconvol11intdouble(xout, n, a1, glen, dxt);
	cconvol11intdouble(yout, n, a1, glen, dyt);
}

//returns curvature and first derivatives
void contour_curvature(int *xc, int*yc, const int n, double *cout, double *xt, double *yt,
	double *a1, double *a2, int glen)
{
	double *xtt = new double[n];
	double *ytt = new double[n];

	//compute first derivative
	contour_derivative(xc, yc, n, xt, yt, a1, glen);

	//compute second derivative
	cconvol11intdouble(xc, n, a2, glen, xtt);
	cconvol11intdouble(yc, n, a2, glen, ytt);

	//compute curvature
	for (int i = 0; i<n; i++)
	{
		cout[i] = ytt[i] * xt[i] - xtt[i] * yt[i];
		double md = xt[i] * xt[i] + yt[i] * yt[i];
		md *= sqrt(md);
		md += 0.00000000001;
		cout[i] /= md;
	}

	delete[] xtt;
	delete[] ytt;
}

/* calculate curvature of a CLOSED contour
* output curvature values as a list
*
* a1[-glen, +glen] - 1st order derivative filter
a2[-glen, +glen] - 2nd order derivative filter
*/
void contour_curvature(int *xc, int*yc, const int n, double *cout, double *a1, double *a2, int glen)
{
	double *xt = new double[n];
	double *yt = new double[n];

	contour_curvature(xc, yc, n, cout, xt, yt, a1, a2, glen);

	delete[] xt;
	delete[] yt;
}

void contour_curvature(int *xc, int*yc, const int n, double *cout, double sig)
{
	/* get smoothing kernel	*/
	double aa1[FLEN_MAX], aa2[FLEN_MAX];
	double *const a1 = &aa1[GLEN_MAX];
	double *const a2 = &aa2[GLEN_MAX];
	int glen = contour_gauss_dkernel(sig, a1, a2);

	contour_curvature(xc, yc, n, cout, a1, a2, glen);
}
