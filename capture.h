
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include "sys/time.h"
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "common.h"


#ifndef V4L2_CID_EXPOSURE_AUTO
//create defines that may be missing from V4L depending on version
#define V4L2_CID_EXPOSURE_AUTO 134217738
#define V4L2_CID_EXPOSURE_ABSOLUTE 134217739
#define V4L2_EXPOSURE_MANUAL 1

#endif

#define FRAME_RATE	30.0
#define GAIN 0 //gain of the camera, 0 - 255
#define BUFFERS_TO_REQUEST 3

#define FIRST_CAM_NUMBER    0   //In case the webcam is plugged in

using namespace std;

class CaptureDevice
{
public:
	CaptureDevice();
	~CaptureDevice();
	bool init_capture(int cam);
	bool init_video(string *filename);
	IplImage* get_frame();
	IplImage* current_frame();
	void initV4L(int cam);

	//reads a new frame from the device
	IplImage* getFrame();
	//close the capture device
	void closeDevice();
	//reads a V4L frame into a buffer
	void getV4LFrame();
	//reads a Video frame
	void getVideoFrame();
	//closes the V4L device
	void closeV4L();
	//converts a V4L buffer into a IplImage for openCV
	void V4LtoOpenCV();
	float getTime();
	void resetTime();

	

private:
	IplImage* frame;
	void enumerate_menu();
v4l2_queryctrl queryctrl;
	CvCapture* camera;
	timeval start, stop;
	float elapsedTime;
	int fd;

	struct buffer {
		void * start;
		size_t length;
	} *buffers;
	int numBufs;
	char checkedOutBuffer;
	v4l2_buffer currBuf;
	CvVideoWriter *writer;
};


#define CLEAR(x) memset (&(x), 0, sizeof (x))
	
