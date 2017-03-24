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

#define NUM_WINDOWS 5 // Must be odd
#define YAW_THRESHOLD 20

// Define local functions:
Mat uv_channels(struct image_t *);
Mat cluster_image(struct image_t *);
uint8_t SearchFloor(Mat);
void write_clusterLabels(Mat);
Mat load_clusterLabels();
Mat yuv422_to_ab(struct image_t *img);
void yuv_to_yuv422(Mat image, char *img);
int *occlusionDetector(Mat img_binary, int ground_id);

/*
 * vision_func(): The function attached to the listener of the v4l2 device.
 * 				: Each time a frame is ready, this function is called to process it.
 */
struct image_t *vision_func(struct image_t *img) {

	printf("[vision_func()] Started\n");

	Mat clusterLabels = cluster_image(img);

	printf("BestLabels retrieved\n");

	FloorID = SearchFloor(Mat clusterLabels);

	printf("I found the floor\n");

	NavWindow = occlusionDetector(img, FloorID);

	printf("Window %d seems the best option\n", NavWindow);

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

		Mat data;
		if (useLab) {
			data    = yuv422_to_ab(img);				// Get ab channels to workable format for K-means
		}
		else {
			data 	= uv_channels(img);				// Get UV channels to workable format for K-means
		}


		// K-means allocation and parameters
		Mat clusterLabels, centers;
	    static Mat init_cluster;
		uint8_t attempts = 1, clusters = 3;
		double eps 			= 0.001;
		printf("cluster init done\n");
		if (init_cluster.empty()){
			kmeans(data, clusters, init_cluster, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, attempts, eps), attempts, KMEANS_PP_CENTERS, centers);
			printf("EMPTY\n");
		}
		kmeans(data, clusters, init_cluster, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, attempts, eps), attempts, KMEANS_USE_INITIAL_LABELS, centers);
		clusterLabels = init_cluster;

		//write_clusterLabels(clusterLabels);

		if(DEBUGGING) {
			Mat img_intermediate(2*clusterLabels.rows,1,CV_32SC1), img_intermediate2, img_seg;
			float scale 		= 255.0/clusters;

			printf("clusterLabels: Rows: %d Cols: %d Channels: %d Type: %d\n" , clusterLabels.rows, clusterLabels.cols, clusterLabels.channels(), clusterLabels.type());

			int index = 0;
			// Restore size of UV:
			for (int i = 0; i < clusterLabels.rows; i++) {
				img_intermediate.at<float>(2*i) 	= clusterLabels.at<float>(i);
				img_intermediate.at<float>(2*i + 1) = clusterLabels.at<float>(i);
				index++;
			}
			printf("Length of restore loop: %d\n", index);


			img_intermediate.convertTo(img_intermediate2, CV_8UC1, scale, 0);
			applyColorMap(img_intermediate2, img_seg, COLORMAP_JET);
			printf("img_seg: Rows: %d Cols: %d Channels: %d Type: %d\n" , img_seg.rows, img_seg.cols, img_seg.channels(), img_seg.type());
			printf("most of debugging working\n");

			cvtColor(img_seg, img_seg, COLOR_BGR2YUV);
			printf("About to yuv2yuv422\n");
			yuv_to_yuv422(img_seg, (char *) img_buf);
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
 * SearchFloor
 * 	- Find which cluster is dominant in the last couple of rows; this cluster is expected to correspond
 *    to the floor
 *
 * Input	: Mat clusterLabels 	- A CV_8UC1 Mat containing labels corresponding to which cluster it
 * 								  	  belongs to, same length as input image.
 * Output	: uint8_t FloorCluster 	- The cluster label (unsigned char) corresponding to the floor
 */

uint8_t SearchFloor(Mat clusterLabels){

	for(int r = clusterLabels.rows - rowScans; r < clusterLabels.rows; r++)
	{
		for(int c = 0; c < clusterLabels.colums; c++){
			if(clusterLabels.at<uchar>(r,c) == 0){
				Environment.Cl0Global++;
			} else if(clusterLabels.at<uchar>(r,c) == 1){
				Environment.Cl1Global++;
			} else Environment.Cl2Global++;
		}
	}

	uchar FloorCluster = 0;
	if(Clusters.Cl1Global>=Clusters.Cl0Global){
			FloorCluster = 1;
		}
		else if(Clusters.Cl2Global>=Clusters.Cl1Global){
			FloorCluster = 2;
		}

	return FloorCluster;
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

Mat yuv422_to_ab(struct image_t *img) {
	uint8_t * img_buf = (uint8_t *) img->buf;

	Mat yuv(img->w*img->h,3,CV_8UC1);
	Mat BGR(img->w*img->h,1,CV_8UC3);
	Mat Lab(img->w*img->h,1,CV_8UC3);
	Mat ab(img->w*img->h,2, CV_8UC1);
	Mat out(img->w*img->h,2, CV_32FC1);
	for (uint16_t y = 0; y < img->h; y++) {
		for (uint16_t x = 0; x < img->w; x += 2) {
			yuv.at<uint8_t>((y*img->w + x)/2, 1) = img_buf[1]; // Y1
			yuv.at<uint8_t>((y*img->w + x)/2, 0) = img_buf[0]; // U
			yuv.at<uint8_t>((y*img->w + x)/2, 1) = img_buf[2]; // V


			yuv.at<uint8_t>((y*img->w + x)/2, 1) = img_buf[3]; // Y2
			yuv.at<uint8_t>((y*img->w + x)/2, 0) = img_buf[0]; // U
			yuv.at<uint8_t>((y*img->w + x)/2, 1) = img_buf[2]; // V
			img_buf += 4;
		}
	}
	yuv.reshape(3,yuv.rows);
	cvtColor(yuv, BGR, CV_YUV2BGR);
	cvtColor(BGR, Lab, CV_BGR2Lab);

	Lab.reshape(1,Lab.rows);
	Lab.col(0).copyTo(ab.col(0));
	Lab.col(1).copyTo(ab.col(1));

	ab.convertTo(out,CV_32F);

	return out;
}

void yuv_to_yuv422(Mat image, char *img) {
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 3);

  int nRows = image.rows;
  int nCols = image.cols;

  uchar *p;
  int index_img = 0;
  p = image.ptr<uchar>(0);
  for (int j = 0; j < nRows*nCols*3; j += 6) {
	img[index_img++] = p[j + 1]; //U
	img[index_img++] = p[j];//Y
	img[index_img++] = p[j + 2]; //V
	img[index_img++] = p[j + 3]; //Y
  }
}

int *occlusionDetector(Mat seg_image, uint8_t ground_id)
{
	/*
	 * Detects the row where the ground transitions to an object for each column and determines the best window.
	 */

	// Best Properties
	int *bestWindow = new int(0); // Current best window ID
	int best_avg = 0; // Current best window average

	// Local Window Properties
	int window_size = seg_image.cols/NUM_WINDOWS; // width of a single window
	int window_avg; // holds window average in loop

	for(int window = 0; window < NUM_WINDOWS; window ++)
	{
		window_avg = 0; // Reset window average

		// Note last few columns won't be iterated. seg_image.cols%NUM_WINDOWS
		for(int c = window*window_size; c < (window+1)*window_size; ++c)
		{
			Mat currCol = seg_image.col(c); // Get current column

			for(int r = currCol.rows-1; r >= 0 ; r--) // Start at bottom row and progressively scan up
			{
				// Tests if the pixel is no longer belonging to the ground in the segmented image
				if (currCol.at<uchar>(r,0) != ground_id)
				{
					window_avg += (seg_image.rows-r)/window_size; // average obstacle height in window;
					break; // Go to next column
				}
			}

		}
		// correction for large yaw angles and to minimise impact of marginal improvements
		window_avg -= YAW_THRESHOLD*abs(window - NUM_WINDOWS/2);

		// Check if current window is better than current best
		if(window_avg > best_avg)
		{
			best_avg = window_avg;
			*bestWindow = window-NUM_WINDOWS/2;
		}

		cout << "Window " << window-NUM_WINDOWS/2 << " average:\t" << window_avg << endl; // print window average

	}

    cout << "Best Window:\t\t"<<	*bestWindow	<< endl; // print best window
	return bestWindow;
}
