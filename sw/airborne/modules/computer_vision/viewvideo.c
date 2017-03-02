/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include "viewvideo.h"
#include "cv.h"
#include "lib/encoding/P7_H264.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

#include "modules/computer_vision/lib/opengl/opengl.h"

#include BOARD_CONFIG


// Downsize factor for video stream
#ifndef VIEWVIDEO_DOWNSIZE_FACTOR
#define VIEWVIDEO_DOWNSIZE_FACTOR 1
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DOWNSIZE_FACTOR)

// From 0 to 99 (99=high)
#ifndef VIEWVIDEO_QUALITY_FACTOR
#define VIEWVIDEO_QUALITY_FACTOR 50
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_QUALITY_FACTOR)

// RTP time increment at 90kHz (default: 0 for automatic)
#ifndef VIEWVIDEO_RTP_TIME_INC
#define VIEWVIDEO_RTP_TIME_INC 0
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_RTP_TIME_INC)

// Default image folder
#ifndef VIEWVIDEO_SHOT_PATH
#ifdef VIDEO_THREAD_SHOT_PATH
#define VIEWVIDEO_SHOT_PATH VIDEO_THREAD_SHOT_PATH
#else
#define VIEWVIDEO_SHOT_PATH /data/ftp/internal_000
#endif
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_SHOT_PATH)

// Define stream framerate
#ifndef VIEWVIDEO_FPS
#define VIEWVIDEO_FPS 30
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_FPS)

// Define stream priority
#ifndef VIEWVIDEO_NICE_LEVEL
#define VIEWVIDEO_NICE_LEVEL 5
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_NICE_LEVEL)

// Check if we are using netcat instead of RTP/UDP
#ifndef VIEWVIDEO_USE_NETCAT
#define VIEWVIDEO_USE_NETCAT FALSE
#endif

#if !VIEWVIDEO_USE_NETCAT && !(defined VIEWVIDEO_USE_RTP)
#define VIEWVIDEO_USE_RTP TRUE
#endif

#ifndef VIEWVIDEO_VERBOSE
#define VIEWVIDEO_VERBOSE 0
#endif

#ifndef VIEWVIDEO_WRITE_VIDEO
#define VIEWVIDEO_WRITE_VIDEO 0
#else
#ifndef VIEWVIDEO_VIDEO_FILE
#define VIEWVIDEO_VIDEO_FILE test
#endif
#endif

#ifndef VIEWVIDEO_STREAM_VIDEO
#define VIEWVIDEO_STREAM_VIDEO 1
#endif

#define printf_debug    if(VIEWVIDEO_VERBOSE > 0) printf

#if VIEWVIDEO_USE_NETCAT
#include <sys/wait.h>
PRINT_CONFIG_MSG("[viewvideo] Using netcat.")
#else
struct UdpSocket video_sock;
PRINT_CONFIG_MSG("[viewvideo] Using RTP/UDP stream.")
PRINT_CONFIG_VAR(VIEWVIDEO_USE_RTP)
#endif

/* These are defined with configure */
PRINT_CONFIG_VAR(VIEWVIDEO_HOST)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT_OUT)

// Initialize the viewvideo structure with the defaults
struct viewvideo_t viewvideo = {
  .is_streaming = FALSE,
  .downsize_factor = VIEWVIDEO_DOWNSIZE_FACTOR,
  .quality_factor = VIEWVIDEO_QUALITY_FACTOR,
#if !VIEWVIDEO_USE_NETCAT
  .use_rtp = VIEWVIDEO_USE_RTP,
#endif
};

static P7_H264_context_t videoEncoder;
#if VIEWVIDEO_WRITE_VIDEO
static FILE *video_file;
#endif

/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
struct image_t *viewvideo_function(struct image_t *img);
struct image_t *viewvideo_function(struct image_t *img)
{
  int32_t h264BufferIndex, size;
  uint8_t* h264Buffer;
  struct image_t releaseImg;

  // Resize image if needed
    struct image_t img_small;
    image_create(&img_small,
                 img->w / VIEWVIDEO_DOWNSIZE_FACTOR,
                 img->h / VIEWVIDEO_DOWNSIZE_FACTOR,
                 IMAGE_YUV422);
  if (viewvideo.is_streaming) {
#if VIEWVIDEO_WRITE_VIDEO || VIEWVIDEO_STREAM_VIDEO
    P7_H264_releaseInputBuffer(&videoEncoder, img->buf_idx);

    while ((h264BufferIndex = P7_H264_find_FreeBuffer(videoEncoder.inputBuffers, BUFFER_TOBE_RELEASED, videoEncoder.numInputBuffers)) != -1){
      releaseImg.buf_idx = h264BufferIndex;
      videoEncoder.inputBuffers[h264BufferIndex].status = BUFFER_FREE;
      v4l2_image_free(VIEWVIDEO_CAMERA.thread.dev, &releaseImg);
    }

    while (!P7_H264_getOutputBuffer(&videoEncoder, &h264BufferIndex))
    {

      h264Buffer = P7_H264_bufferIndex2OutputPointer(&videoEncoder, h264BufferIndex);
      size = P7_H264_bufferIndex2OutputSize(&videoEncoder, h264BufferIndex);


      if (size == 0)
        fprintf(stderr, "%s:%d warning, no data to write\n",__FILE__,__LINE__);
      else {
    	printf_debug("Got frame of size: %d\r\n", size);
    	printf_debug("Byte: %2X %2X %2X %2X %2X\n", h264Buffer[0],h264Buffer[1], h264Buffer[2], h264Buffer[3], h264Buffer[4]);
#if VIEWVIDEO_STREAM_VIDEO
    	rtp_frame_send_h264(&video_sock, h264Buffer, size);
#endif
#if VIEWVIDEO_WRITE_VIDEO
        fwrite(h264Buffer, size, 1, video_file);
#endif
      }
      P7_H264_releaseOutputBuffer(&videoEncoder, h264BufferIndex);
    }
#endif
  } else {
    v4l2_image_free(VIEWVIDEO_CAMERA.thread.dev, img);
  }
  return NULL; // No new images were created
}

/**
 * Initialize the view video
 */
void viewvideo_init(void)
{
#if VIEWVIDEO_WRITE_VIDEO
  char video_name[512];
  sprintf(video_name, "%s/%s.h264", STRINGIFY(VIEWVIDEO_SHOT_PATH), STRINGIFY(VIEWVIDEO_VIDEO_FILE));
  int tries = 0;
  int maxtries = 5;
  do {
	  video_file = fopen(video_name, "w");
	  tries++;
  } while(video_file == NULL && tries < maxtries);
  if(video_file == NULL)
  {
	  printf("[viewvideo] Failed to create .h264 file.\n");
  }
#endif
  struct video_listener *listener = cv_add_to_device(&VIEWVIDEO_CAMERA, viewvideo_function);
  listener->maximum_fps = 0;

#if VIEWVIDEO_WRITE_VIDEO || VIEWVIDEO_STREAM_VIDEO
  viewvideo.is_streaming = true;
  videoEncoder.inputType = H264ENC_YUV422_INTERLEAVED_UYVY;
#endif
#if VIEWVIDEO_WRITE_VIDEO
  videoEncoder.bitRate   = 6*8*1000*1000; // 6 MBps
#else
  videoEncoder.bitRate   = 1*1000*1000; // 1 Mbps
#endif
#if VIEWVIDEO_WRITE_VIDEO || VIEWVIDEO_STREAM_VIDEO
  videoEncoder.frameRate = VIEWVIDEO_FPS;
  videoEncoder.intraRate = VIEWVIDEO_FPS;
  P7_H264_open(&videoEncoder, VIEWVIDEO_CAMERA.thread.dev);
#endif
  // Open udp socket
#if VIEWVIDEO_STREAM_VIDEO
  char save_name[512];
  udp_socket_create(&video_sock, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST);
  udp_socket_set_sendbuf(&video_sock, 1500*10);
  // Create an SDP file for the streaming
  sprintf(save_name, "%s/stream.sdp", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
	  fprintf(fp, "v=0\n");
	  fprintf(fp, "m=video %d RTP/AVP 96\n", (int)(VIEWVIDEO_PORT_OUT));
	  fprintf(fp, "a=rtpmap:96 H264\n");
	  fprintf(fp, "a=framerate:%d\n", (int)(VIEWVIDEO_FPS));
	  fprintf(fp, "c=IN IP4 0.0.0.0\n");
	  fclose(fp);
  } else {
	  printf_debug("[viewvideo] Failed to create SDP file.\n");
  }
#endif
}
