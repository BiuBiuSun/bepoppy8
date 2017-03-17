/*
 * OpenCV Module for Bepoppy8
 */


#define BOARD_CONFIG "boards/bebop.h"

#include "bepoppy8_cv.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;


struct image_t *vision_func(struct image_t *img) {
	/*
	int32_t uv_length 	= (img->h)*(img->w)/2;

	int8_t *source[uv_length*4] = (int8_t) img->buf;

	Mat uv(uv_length,2, CV_8UC3, 0);

	int index = 0;
	for (int row = 0; row < (img->h); row++) { // Loop over rows:
		for (int col = 0; col < (img->w)/2; col++) { // Loop over collumns:
			uv[row][1] = source[index];	index += 2;
			uv[row][2] = source[index];	index += 2;
		}
	}
	*/

  return img;
}
