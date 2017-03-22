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

// Define local fucions:
Mat uv_channels(struct image_t *);
Mat cluster_image(struct image_t *);
void setNavigationParams(struct image_t *, Mat);




/*
 * vision_func(): The function attached to the listener of the v4l2 device.
 * 				: Each time a frame is ready, this function is called to process it.
 */
struct image_t *vision_func(struct image_t *img) {

	Mat clusterLabels = cluster_image(img);
	//setNavigationParams(img, clusterLabels);

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
		uint8_t *img_buf 	= (uint8_t *) img->buf;			// Get image buffer
		Mat uv 				= uv_channels(img);				// Get UV channels to workable format for K-means

		// K-means allocation and parameters
		Mat clusterLabels, centers;
		uint8_t attempts = 3, clusters = 3;
		double eps 			= 0.001;

		kmeans(uv, clusters, clusterLabels, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, attempts, eps), attempts, KMEANS_PP_CENTERS, centers);

		if(DEBUGGING) {
			Mat img_intermediate, img_seg;
			float scale 		= 255.0/clusters;

			clusterLabels.convertTo(img_intermediate, CV_8UC1, scale, 0);
			applyColorMap(img_intermediate, img_seg, COLORMAP_JET);

			colorrgb_opencv_to_yuv422(img_seg, (char *) img_buf);
		}

	return clusterLabels;
}

/*
 * Local function to allocate the UV channels to a format k-means is able to process.
 */
Mat uv_channels(struct image_t *img) { // TODO: Investigate possible performance gain using: .split(uv Y) transpose(uv) reshape(1,2) transpose(uv)
	uint8_t *img_buf = (uint8_t *) img->buf;

	Mat uv(img->h*img->w*0.5,2,CV_32FC1);

	int index = 0;
	for (int m = 0; m < img->h; m++) {
		for (int n = 0; n < img->w; n += 2) {
			uv.at<float>(m*img->h + n, 0) 	= (float) img_buf[index]; index += 2;
			uv.at<float>(m*img->h + n, 1) 	= (float) img_buf[index]; index += 2;
		}
	}

	return uv;
}

/*
 * Navigation Parameters
 * 	- Sets parameters used in the periodic function to navigate through the CyberZoo
 *
 * Inputs	: uint32_t img_width, img_height 	- The height and width of the image in pixels.
 * 			: Mat bestLabels 					- A CV_8UC1 Mat containing labels corresponding to which cluster it belongs to, same length as input image.
 * Outputs	: 									- Sets global variables, so periodic function can use those to determine its actions.
 */
void setNavigationParams(struct image_t *img, Mat clusterLabels) {
	// TODO: Make thread safe!

	struct ArrayInfo NavWindowInfo;
	uint8_t element;

	NavWindowInfo.ArrayLength 	= (img->h)*0.5;
	NavWindowInfo.InitPoint 	= (img->h)*(img->w)*0.5-NavWindowInfo.ArrayLength;

	uint8_t WindowReference 	= NavWindowInfo.InitPoint + NavWindowInfo.ArrayLength/2;
	struct Window AvoidWindow 	= {WindowReference-WindowHalfSize, WindowReference+WindowHalfSize};

	Environment = {0,0,0,0,0,0,0,0,0};

	for(element = NavWindowInfo.InitPoint; NavWindowInfo.ArrayLength; element++){

		if(clusterLabels.at<int>(element)==0){
			Environment.Cl1Global++;
			if(element >= AvoidWindow.LeftBoundary && element <= WindowReference){
				Environment.Cl1AvoidLeft++;
			} else if(element >= WindowReference && element <= AvoidWindow.RightBoundary){
				Environment.Cl1AvoidRight++;
			}
		}
		else if(clusterLabels.at<int>(element)==1){
			Environment.Cl2Global++;
			if(element >= AvoidWindow.LeftBoundary && element <= WindowReference){
				Environment.Cl2AvoidLeft++;
			} else if(element >= WindowReference && element <= AvoidWindow.RightBoundary){
				Environment.Cl2AvoidRight++;
			}
		}
		else if(clusterLabels.at<int>(element)==2){
			Environment.Cl3Global++;
			if(element >= AvoidWindow.LeftBoundary && element <= WindowReference){
				Environment.Cl3AvoidLeft++;
			} else if(element >= WindowReference && element <= AvoidWindow.RightBoundary){
				Environment.Cl3AvoidRight++;
			}
		} else continue;

	}

}












