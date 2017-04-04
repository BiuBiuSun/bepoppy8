/*
 * OpenCV Module for Bepoppy8
 */


#define BOARD_CONFIG "boards/bebop.h"
#include "bepoppy8_cv.h"
#include "modules/computer_vision/opencv_image_functions.h"
#include <iostream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <numeric>
using namespace cv;


// Define prototype functions that rely on C++ here:
Mat 	cluster_image(struct image_t *);
uint8_t SearchFloor(Mat, struct image_t *);
int8_t 	windowSearch(Mat clusterLabels, uint8_t FloorCluster, struct image_t *img);

void 	write_clusterLabels(Mat);
Mat 	load_clusterLabels();

Mat 	uv_channels(struct image_t *);
Mat 	yuv422_to_ab(struct image_t *img);
void 	yuv_to_yuv422(Mat image, char *img);


// Global Variables:
uint8_t attempts 		= 3;
uint8_t clusters 		= 3;
uint8_t rowScans 		= 5;
double eps 				= 0.01;



/*
 * vision_func(): The function attached to the listener of the v4l2 device.
 * 				: Each time a frame is ready, this function is called to process it.
 */
struct image_t *vision_func(struct image_t *img) {

	Mat clusterLabels 	= cluster_image(img);					// Process Image
	uint8_t FloorID 	= SearchFloor(clusterLabels, img);		// Find floor of the image

	pthread_mutex_lock(&navWindow_mutex);
	    {
		NavWindow = windowSearch(clusterLabels, FloorID, img);	// Find and set best window for navigation
	    }
	pthread_mutex_unlock(&navWindow_mutex);

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
		Mat clusterLabels, centers, data;
		static Mat init_cluster;

		// Sort data based on YUV or CIE Lab format:
		if (useLab) {
			data    	= yuv422_to_ab(img); 							// Get ab channels to workable format for K-means
		}
		else {
			data 		= uv_channels(img); 							// Get UV channels to workable format for K-means
		}

		// Check if clusters are initialized:
		if (init_cluster.empty()){
			kmeans(data, clusters, init_cluster, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, attempts, eps), attempts, KMEANS_PP_CENTERS, centers);
		}
		kmeans(data, clusters, init_cluster, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, attempts, eps), attempts, KMEANS_USE_INITIAL_LABELS, centers);
		clusterLabels = init_cluster;

		// Visualize clusters by modifying the image buffer
		if(VISUALIZE) {
			Mat img_intermediate(2*clusterLabels.rows,1,CV_32SC1), img_intermediate2, img_seg;
			float scale 		= 255.0/clusters;

			// Restore clustered image size to same length as the image buffer
			int index = 0;
			for (int i = 0; i < clusterLabels.rows; i++) {
				img_intermediate.at<float>(2*i) 	= clusterLabels.at<float>(i);
				img_intermediate.at<float>(2*i + 1) = clusterLabels.at<float>(i);
				index++;
			}

			// Convert clustered image to YUV422 and write back to buffer
			img_intermediate.convertTo(img_intermediate2, CV_8UC1, scale, 0); 		// Scale for colormapping, and convert to 8bit integer
			applyColorMap(img_intermediate2, img_seg, COLORMAP_BONE);				// Apply greyscale colormap
			cvtColor(img_seg, img_seg, COLOR_BGR2YUV); 								// Convert BGR to YUV

			yuv_to_yuv422(img_seg, (char *) img_buf); 								// Convert YUV to YUV422 and write result to image buffer
		}

	return clusterLabels;
}


/*
 * SearchFloor
 * 	- Find which cluster is dominant in the last couple of rows; this cluster is expected to correspond
 *    to the floor
 *
 * Input	: Mat clusterLabels 	- A CV_8UC1 Mat containing labels corresponding to which cluster it
 * 								  	  belongs to, same length as input image.
 * 			: struct image_t *img 	- Pointer to the processed image
 * Output	: uint8_t FloorCluster 	- The cluster label (unsigned char) corresponding to the floor
 */
uint8_t SearchFloor(Mat clusterLabels, struct image_t *img){
	struct ClusterInfo Environment = {0,0,0}; 		// Initialize counter of labels

	// Count the number of occurrences of labels in the bottom rows
	for(int c = 0; c < img->h; c++) {
		int start = c * img->w/2;
		for(int i = start; i < start + rowScans; i++){

			if(clusterLabels.at<uint32_t>(i) == 0){
				Environment.Cl0Global++;
			}
			if(clusterLabels.at<uint32_t>(i) == 1){
				Environment.Cl1Global++;
			}
			if(clusterLabels.at<uint32_t>(i) == 2){
				Environment.Cl2Global++;
			}
		}
	}

	// Find most probable floor label
	uint8_t FloorCluster = 0;
	if(Environment.Cl1Global>=Environment.Cl0Global && Environment.Cl1Global>=Environment.Cl2Global){
			FloorCluster = 1;
		}
	else if(Environment.Cl2Global>=Environment.Cl1Global && Environment.Cl2Global>=Environment.Cl0Global){
			FloorCluster = 2;
		}

	return FloorCluster;
}

/*
 * Detects the row where the ground transitions to an object for each column and determines the best window.
 */
int8_t windowSearch(Mat clusterLabels, uint8_t FloorCluster, struct image_t *img){
	Mat inde_line(1, img->h, CV_32FC1);
	Mat binary_img = clusterLabels == FloorCluster;

	// Find first non-floor pixel relative to the bottom of the image, for each column of the image.
	for(int j=0; j<img->h; j++){
		for(int i=j*img->w/2; i<img->w/2*(j+1); i++){
				if(binary_img.at<uchar>(i)==0){
					inde_line.at<float>(j) =i - j*img->w/2;
					break;
				}
				else{
					inde_line.at<float>(j) = img->w/2;
				}
		}
	}

	// Find the average height of the floor for each window.
	float windowsAverage[NumWindows];
	if ((NumWindows % 2) && (inde_line.cols*inde_line.rows % NumWindows == 0)) {
			int windowSize = inde_line.cols*inde_line.rows/NumWindows;

				for (int i = 0; i < NumWindows; i++) {
					windowsAverage[i] = 0;
					for (int j = 0; j < windowSize; j++) {
						windowsAverage[i] += inde_line.at<float>(i*windowSize + j)/windowSize;
					}
				}
	}

	// Loop over windows, and determine best option window to move to
	int8_t bestWindow = 0;
	float maxAverage = windowsAverage[0];
	for(uint8_t i = 1; i < NumWindows; i++){
		if(windowsAverage[i] > maxAverage){
			maxAverage = windowsAverage[i];
			bestWindow = i;
		}
	}


	// Check minimum threshold, if not met perform 180 degree turn
	if (maxAverage < windowThreshold) {
		printf("\nThreshold! Turning 180 degrees! \n\n");
		return 9;
	}

	return (bestWindow - (NumWindows-1)/2); 				// Make windows range from [-2,2]
}


/*
 * Write the clusterLabels file to .xml file for post processing and navigation testing.
 */
void write_clusterLabels(Mat clusterLabels) {

	// write Mat to file
	FileStorage fs("file.xml", FileStorage::WRITE);
	fs << "clusterLabels" << clusterLabels;

	fs.release();

}


/*
 * Load clusterLabels from .xml file
 */
Mat load_clusterLabels() {
	Mat clusterLabels;

	// read Mat from file
	FileStorage fs("file.xml", FileStorage::READ);
	fs["clusterLabels"] >> clusterLabels;

	fs.release();

	return clusterLabels;
}


/*
 * Allocate the UV channels to a format k-means is able to process.
 */
Mat uv_channels(struct image_t *img) {
	uint8_t *img_buf 		= (uint8_t *) img->buf;			// Read image buffer
	Mat uv(img->h*img->w*0.5,2,CV_32FC1); 					// Initialize uv

	// Sort the buffer in format suitable for K-means [u v], with u and v column vectors
	for (uint16_t y = 0; y < img->h; y++) {
	    for (uint16_t x = 0; x < img->w; x += 2) {
	    	uv.at<float>((y*img->w + x)/2, 0) = img_buf[0]; // U
	    	uv.at<float>((y*img->w + x)/2, 1) = img_buf[2]; // V
	    	img_buf += 4;
	    }
	}

	return uv;
}


/*
 * Convert YUV to CIE Lab and allocate the channel to a format K-means is able to process.
 */
Mat yuv422_to_ab(struct image_t *img) {
	uint8_t * img_buf = (uint8_t *) img->buf;

	// Initialize all Mat structures
	Mat yuv(img->w*img->h/2,1,CV_8UC3);
	Mat BGR(img->w*img->h/2,1,CV_8UC3);
	Mat Lab(img->w*img->h/2,1,CV_8UC3);
	Mat Lab2[3];
	Mat ab(img->w*img->h/2,2, CV_8UC1);
	Mat out(img->w*img->h/2,2, CV_32FC1);

	// Loop over buffer and reformat to 3 channel YUV
	int index = 0;
	for (uint16_t y = 0; y < img->h; y++) {
		for (uint16_t x = 0; x < img->w; x += 2) {

			Vec3b& elem 	= yuv.at<Vec3b>(index++);
			elem[0] 		= 0.5*(img_buf[1] + img_buf[3]); 	// Y   	: Average Y since, it is measured twice as often as UV.
			elem[1] 		= img_buf[0];						// U	:
			elem[2] 		= img_buf[2];						// V 	:

			img_buf += 4;
		}
	}

	// Convert from YUV to CIE Lab
	cvtColor(yuv, BGR, CV_YUV2BGR);
	cvtColor(BGR, Lab, CV_BGR2Lab);

	// Reformat to workable format for K-means
	split(Lab, Lab2);
	hconcat(Lab2[0],Lab2[1],out);
	out.convertTo(out,CV_32FC1);

	return out;
}


/*
 * Convert YUV to YUV422 and write result to image buffer
 */
void yuv_to_yuv422(Mat image, char *img) {
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 3);

  int nRows = image.rows;
  int nCols = image.cols;

  uchar *p;
  int index_img = 0;
  p = image.ptr<uchar>(0);
  for (int j = 0; j < nRows*nCols*3; j += 6) {
	img[index_img++] = p[j + 1]; 	//U
	img[index_img++] = p[j];		//Y
	img[index_img++] = p[j + 2]; 	//V
	img[index_img++] = p[j + 3]; 	//Y
  }
}

