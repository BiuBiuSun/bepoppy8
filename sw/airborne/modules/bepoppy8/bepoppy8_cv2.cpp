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


// Declare local functions:
Mat cluster_image(struct image_t *img);
void setNavigationParam(struct image_t *img, Mat bestLabels);

// Function run from vision thread:
struct image_t *vision_func(struct image_t *img) {

		Mat bestLabels = cluster_image(img);
		setNavigationParam(img, bestLabels);

  return img;
}

/*
 * K-means clustering
 * 	- Uses the K-means algorithm to cluster the image from the bebop camera.
 *
 * Input	: struct image_t *	- Image structure, containing info and the image buffer.
 * Output	: Mat bestLabels 	- A CV_8UC1 Mat containing labels corresponding to which cluster it belongs to, same length as input image.
 * 			:  					- Image buffer is set to the segmented image.
 */
Mat cluster_image(struct image_t *img) {
		char *img_buf 		= img->buf;						// Read out image buffer

		Mat yuv((img->w), (img->h), CV_8UC2, img_buf);		// Set Mat as 2 channel image with data from buffer
		Mat img_bgr, bestLabels, centers, img_seg;			// Initialize Mat variables

		// K-means Parameters:
		uint8_t attempts 	= 2, clusters = 3;
		double eps 			= 0.001;
		float scale 		= 255.0/clusters;

		cvtColor(yuv, img_bgr, CV_YUV2BGR_Y422);			// Convert YUV422 to BGR (OpenCV format)

		img_bgr = img_bgr.reshape(1, (img->h)*(img->w)); 	// Reshape Nx1x(3 Channels) to a Nx3 column vector
		img_bgr.convertTo(img_bgr, CV_32FC1);				// Convert CV_8UC1 to CV_32FC1, as is required by kmeans()

		// Cluster:
		kmeans(img_bgr, clusters, bestLabels, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, attempts, eps), attempts, KMEANS_PP_CENTERS, centers);


		bestLabels.convertTo(bestLabels, CV_8UC1, scale,0);	// Scale labels to uint8 range in preparation for colormap.
		applyColorMap(bestLabels, img_seg, COLORMAP_JET);	// map bestLabels to create color image.

		colorrgb_opencv_to_yuv422(img_seg, img_buf); 		// Set buffer to clustered image

	return bestLabels;
}

/*
 * Navigation Parameters
 * 	- Sets parameters used in the periodic function to navigate through the CyberZoo
 *
 * Inputs	: uint32_t img_width, img_height 	- The height and width of the image in pixels.
 * 			: Mat bestLabels 					- A CV_8UC1 Mat containing labels corresponding to which cluster it belongs to, same length as input image.
 * Outputs	: 									- Sets global variables, so periodic function can use those to determine its actions.
 */
void setNavigationParam(uint32_t img_width, uint32_t img_height, Mat bestLabels) {
	// TODO: Make thread safe!

	struct ArrayInfo NavWindowInfo;

	NavWindowInfo.ArrayLength 	= img_width/2;											// TODO: Check if dividing is still needed.
	NavWindowInfo.InitPoint 	= img_width*img_height/2-NavWindowInfo.ArrayLength; 	// TODO: Check if dividing is still needed.


	uint8_t WindowReference 	= NavWindowInfo.InitPoint + NavWindowInfo.ArrayLength/2;
	WindowHalfSize 				= 40; 													// TODO: Move to bepoppy8_init() function
	struct Window AvoidWindow = {WindowReference-WindowHalfSize, WindowReference+WindowHalfSize};

	Environment = {0,0,0,0,0,0};

	for(uint8_t element = NavWindowInfo.InitPoint; NavWindowInfo.ArrayLength; element++){

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

}
