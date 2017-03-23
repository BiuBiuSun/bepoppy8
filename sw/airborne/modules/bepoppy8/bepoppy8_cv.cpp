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

// Define local functions:
Mat uv_channels(struct image_t *);
Mat cluster_image(struct image_t *);
void setNavigationParams(struct image_t *, Mat);
void write_clusterLabels(Mat);
Mat load_clusterLabels();



/*
 * vision_func(): The function attached to the listener of the v4l2 device.
 * 				: Each time a frame is ready, this function is called to process it.
 */
struct image_t *vision_func(struct image_t *img) {
	printf("[vision_func()] Started\n");
	Mat clusterLabels = cluster_image(img);
	//setNavigationParams(img, clusterLabels);
	printf("[vision_func()] Finished\n");
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
		printf("[cluster_image()] Start\n");
		uint8_t *img_buf 	= (uint8_t *) img->buf;			// Get image buffer
		printf("cluster imgbuf\n");
		Mat uv 				= uv_channels(img);				// Get UV channels to workable format for K-means

		// K-means allocation and parameters
		Mat clusterLabels, centers;
		uint8_t attempts = 1, clusters = 3;
		double eps 			= 0.001;
		printf("cluster init done\n");
		kmeans(uv, clusters, clusterLabels, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, attempts, eps), attempts, KMEANS_PP_CENTERS, centers);

		//write_clusterLabels(clusterLabels);

		if(DEBUGGING) {
			Mat img_intermediate(2*clusterLabels.rows,1,CV_32SC1), img_intermediate2, img_seg;
			float scale 		= 255.0/clusters;

			printf("clusterLabels: Rows: %d Cols: %d Channels: %d Type: %d" , clusterLabels.rows, clusterLabels.cols, clusterLabels.channels(), clusterLabels.type());

			// Restore size of UV:
			for (int i = 0; i < clusterLabels.rows; i++) {
				img_intermediate.at<float>(2*i) 	= clusterLabels.at<float>(i);
				img_intermediate.at<float>(2*i + 1) = clusterLabels.at<float>(i);
			}


			img_intermediate.convertTo(img_intermediate2, CV_8UC1, scale, 0);
			applyColorMap(img_intermediate2, img_seg, COLORMAP_JET);
			printf("most of debugging working\n");

			colorrgb_opencv_to_yuv422(img_seg, (char *) img_buf);
		}
		printf("[cluster_image()] Finished\n");
	return clusterLabels;
}

/*
 * Local function to allocate the UV channels to a format k-means is able to process.
 */
Mat uv_channels(struct image_t *img) { // TODO: Investigate possible performance gain using: .split(uv Y) transpose(uv) reshape(1,2) transpose(uv)
	printf("[uv_channels()] Started\n");
	uint8_t *img_buf = (uint8_t *) img->buf;
	printf("[uv_channels()] 1\n");
	Mat uv(img->h*img->w*0.5,2,CV_32FC1); // reshape to 2 rows
	printf("[uv_channels()]2\n");

	printf("%d\n",img->h);

	for (uint16_t y = 0; y < img->h; y++) {
	    for (uint16_t x = 0; x < img->w; x += 2) {
	    	uv.at<float>((y*img->w + x)/2, 0) = img_buf[0]; // U
	    	uv.at<float>((y*img->w + x)/2, 1) = img_buf[2]; // V
	    	img_buf += 4;
	    }

	}

	printf("[uv_channels()] Done\n");
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
	WindowHalfSize 				= 40; // TODO: Move to init?
	struct Window AvoidWindow 	= {WindowReference-WindowHalfSize, WindowReference+WindowHalfSize};

	Environment = {0,0,0,0,0,0,0,0,0};

	for(element = NavWindowInfo.InitPoint; element < NavWindowInfo.ArrayLength; element++){

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

/*
 * Write the clusterLabels file to .txt files for post processing and nav testing.
 */
void write_clusterLabels(Mat clusterLabels) {

	// write Mat to file
	FileStorage fs("file.xml", FileStorage::WRITE);
	fs << "clusterLabels" << clusterLabels;

	fs.release();

}

Mat load_clusterLabels() {
	Mat clusterLabels;

	// read Mat from file
	FileStorage fs("file.xml", FileStorage::READ);
	fs["clusterLabels"] >> clusterLabels;

	fs.release();

	return clusterLabels;
}
