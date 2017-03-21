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

	uint8_t *img_buf 	= (uint8_t *) img->buf;
	int32_t uv_length 	= (img->h)*(img->w)/2;
	uint8_t attempts 	= 5, clusters = 3;

	Mat uv(uv_length,2, CV_32F), bestLabels, centers, img_rgb, img_out(uv_length*2,1, CV_8UC3);

	double eps 			= 0.001;
	float scale 		= 255.0/clusters;


	int index = 0;
	struct ArrayInfo NavWindowInfo;

	for (int row = 0; row < (img->w); row++) { // Loop over rows:
		for (int col = 0; col < (img->h)/2; col++) { // Loop over columns:
			uv.at<float>(col,0) = (float) img_buf[index];	index += 2;
			uv.at<float>(col,1) = (float) img_buf[index];	index += 2;
		}
	}

	NavWindowInfo.ArrayLength = (img->h)/2;
	NavWindowInfo.InitPoint = uv_length-NavWindowInfo.ArrayLength;

	kmeans(uv, clusters, bestLabels, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, attempts, eps), attempts, KMEANS_PP_CENTERS, centers);

	uint8_t element;

	uint8_t WindowReference = NavWindowInfo.InitPoint + NavWindowInfo.ArrayLength/2;
	WindowHalfSize = 40;
	struct Window AvoidWindow = {WindowReference-WindowHalfSize, WindowReference+WindowHalfSize};

	Environment = {0,0,0,0,0,0};

	for(element = NavWindowInfo.InitPoint; NavWindowInfo.ArrayLength; element++){

		if(bestLabels.at<int>(element)==0){
			Environment.Cl1Global++;
			if(element >= AvoidWindow.LeftBoundary && element <= WindowReference){
				Environment.Cl1AvoidLeft++;
			} else if(element >= WindowReference && element <= AvoidWindow.RightBoundary){
				Environment.Cl1AvoidRight++;
			}
		}
		else if(bestLabels.at<int>(element)==1){
			Environment.Cl2Global++;
			if(element >= AvoidWindow.LeftBoundary && element <= WindowReference){
				Environment.Cl2AvoidLeft++;
			} else if(element >= WindowReference && element <= AvoidWindow.RightBoundary){
				Environment.Cl2AvoidRight++;
			}
		}
		else if(bestLabels.at<int>(element)==2){
			Environment.Cl3Global++;
			if(element >= AvoidWindow.LeftBoundary && element <= WindowReference){
				Environment.Cl3AvoidLeft++;
			} else if(element >= WindowReference && element <= AvoidWindow.RightBoundary){
				Environment.Cl3AvoidRight++;
			}
		} else continue;

	}


	/*
	bestLabels.convertTo(bestLabels, CV_8UC1, scale,0);
	applyColorMap(bestLabels, img_rgb, COLORMAP_JET);


	for (int i = 0; i < uv_length; i++) {
		img_out.at<uint8_t>(2*i,0), img_out.at<uint8_t>(2*i+1,0) = img_rgb.at<uint8_t>(i,0);
	}

	// img_out.reshape(0, img->w); Not needed if my reasoning is correct


	colorrgb_opencv_to_yuv422(img_out,(char *) img_buf); // Set buffer to clustered image */

  return img;
}
