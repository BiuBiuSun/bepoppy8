/*
 * OpenCV Module for Bepoppy8
 */


#define BOARD_CONFIG "boards/bebop.h"
#include "bepoppy8_cv.h"
#include "modules/computer_vision/opencv_image_functions.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
using namespace cv;


struct image_t *vision_func(struct image_t *img) {

	char *img_buf 		= img->buf;
	uint8_t attempts 	= 2, clusters = 3;

	Mat yuv((img->w), (img->h), CV_8UC2, img_buf);
	Mat img_bgr, bestLabels, centers, img_seg;

	double eps 			= 0.001;
	float scale 		= 255.0/clusters;

	cvtColor(yuv, img_bgr, CV_YUV2BGR_Y422);

	img_bgr = img_bgr.reshape(1, (img->h)*(img->w)); // change to a Nx3 column vector
	img_bgr.convertTo(img_bgr, CV_32FC1);

	kmeans(img_bgr, clusters, bestLabels, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, attempts, eps), attempts, KMEANS_PP_CENTERS, centers);

	// bestLabels.convertTo(bestLabels, CV_8UC1, scale,0); // Needed?
	applyColorMap(bestLabels, img_seg, COLORMAP_JET);

	colorrgb_opencv_to_yuv422(img_seg, img_buf); // Set buffer to clustered image

  return img;
}
