#include "capture.h"
#include "common.h"

using namespace std;


CaptureDevice::CaptureDevice()
{
	gettimeofday(&start, NULL);

#ifdef GRAYSCALE
	frame = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),8,1);
#else
	frame = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),8,3);
#endif
	fd=-1;
	CLEAR(currBuf);
	currBuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	currBuf.memory = V4L2_MEMORY_MMAP;
	checkedOutBuffer = -1;


	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);

	return;
}

CaptureDevice::~CaptureDevice()
{
	gettimeofday(&start, NULL);

#ifdef USE_VIDEO
        cvReleaseCapture(&camera);
#else
        closeDevice();
#endif


	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
	return;
}

bool CaptureDevice::init_capture(int cam)
{
	gettimeofday(&start, NULL);
        
#ifdef USE_VIDEO

    char filename1[20];

    sprintf(filename1,VIDEO_FILENAME,cam);
    string filename(filename1);
    init_video(&filename);
#else
	initV4L(cam);
#endif

#ifdef SAVE_VIDEO
        //Set up video output
        char output[20];

        sprintf(output,"cam%d.avi",cam);
	writer = cvCreateVideoWriter(output, CV_FOURCC('M', 'J', 'P', 'G'),30,cvSize(PIC_WIDTH,PIC_HEIGHT) );
#endif

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);


		return true;
}

bool CaptureDevice::init_video(string *filename)
{

	if(camera = cvCaptureFromAVI(filename->c_str()))
	{	
		cout << "Opened video file\n";
		return true;
	}
	else
	{	
		cout << "Could not open video file\n";
		return false;
	}
}

IplImage* CaptureDevice::get_frame()
{
	gettimeofday(&start, NULL);


	IplImage* tempPic;
        #ifdef USE_VIDEO
                getVideoFrame();
	#else
		getV4LFrame();
	#endif

//Contrast stretch first to make sure it's good
	//double minVal;
	//double maxVal;

	//cvMinMaxLoc(frame, &minVal,&maxVal,NULL);
	//cvConvertScale(frame, frame, 255.0/(maxVal-minVal), -minVal/(maxVal-minVal) );

#ifdef SAVE_VIDEO
        IplImage* outImage = cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,3);
        cvCvtColor(frame,outImage,CV_GRAY2RGB);

	cvWriteFrame(writer,outImage);
        cvReleaseImage(&outImage);
#endif

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
	
	return frame;

}

IplImage* CaptureDevice::current_frame()
{
	return frame;
}

void CaptureDevice::initV4L(int cam)
{
	gettimeofday(&start, NULL);

        
	cout<<"opening video device"<<endl;
	v4l2_capability cap;
	char cam_num[1];
	sprintf(cam_num,"%d",cam);
	std:string dev_path;
	dev_path.append("/dev/video");
	dev_path.append(cam_num);

//	fd = open("/dev/video0", O_RDWR | O_NONBLOCK, 0);	
//	fd = open("/dev/video0", O_RDWR, 0);
	fd = open(dev_path.c_str(), O_RDWR, 0);
	if (-1 ==ioctl(fd, VIDIOC_QUERYCAP, &cap))
	{
		cout<<"failed to get capability"<<endl;
		exit(1);
	}
	cout<<cap.card<<" - "<<cap.capabilities<<endl;

//list all available formats
	v4l2_fmtdesc fmtdesc;
	CLEAR(fmtdesc);
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmtdesc.index=0;
	do
	{
		if (-1==ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
		{
			if (errno!=EINVAL)
				cout<<"error reading enumerated formats"<<endl;
			break;
		}
		cout<<fmtdesc.index<<":    "<<fmtdesc.description<<": "<<fmtdesc.pixelformat<<" compressed: "<<fmtdesc.flags<<endl;
		fmtdesc.index++;
	}while(1);

//set the gain to 200
	struct v4l2_control c;
	c.id = V4L2_CID_GAIN;
	c.value=250;
	if (-1 == ioctl(fd, VIDIOC_S_CTRL, &c))
	{
		cout<<"failed to set gain via standard controls"<<endl;
		switch(errno)
		{
		case EINVAL:
			cout<<"invalud id"<<endl;
			break;
		case ERANGE:
			cout<<"value out of range"<<endl;
			break;
		case EBUSY:
			cout<<"control is unavailable"<<endl;
			break;
		default:
			cout<<"unknown ioctl error: "<<errno<<endl;
		};
	}


//list available controls
	CLEAR(queryctrl);
	for (queryctrl.id=V4L2_CID_BASE;
	     queryctrl.id < V4L2_CID_LASTP1;
	     queryctrl.id++)
	{
		if (0==ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl))
		{
			if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				continue;
			cout <<"Control "<<queryctrl.name<<endl;
			if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
				enumerate_menu();
		}
		else
		{
			if (errno == EINVAL)
				continue;
			cout<<"error querying control"<<endl;
			exit(1);
		}
	}

	for (queryctrl.id = V4L2_CID_PRIVATE_BASE;;queryctrl.id++)
	{
		if (0==ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl))
		{
			if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				continue;
			cout<<"Control "<<queryctrl.name<<endl;
			if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
				enumerate_menu();
		}
		else
		{
			if (errno == EINVAL)
				break;
			cout <<"error querrying private controls"<<endl;
			exit(1);
		}
	}

//set the output format
	cout<<"setting output format"<<endl;
	struct v4l2_format fmt;
	CLEAR(fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = PIC_WIDTH;
	fmt.fmt.pix.height = PIC_HEIGHT;
//fisrt read the format back, then modify its settings
	if (-1 == ioctl(fd, VIDIOC_G_FMT, &fmt))
	{
		cout<<"failed to read format"<<endl;
		exit(1);
	}
	cout<<"type: "<<fmt.type<<"  width:"<<fmt.fmt.pix.width<<"  height:"<<fmt.fmt.pix.height<<" pix:"<<fmt.fmt.pix.pixelformat<<endl;
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = PIC_WIDTH;
	fmt.fmt.pix.height = PIC_HEIGHT;
	fmt.fmt.pix.pixelformat = 1448695129;
	cout<<"type: "<<fmt.type<<"  width:"<<fmt.fmt.pix.width<<"  height:"<<fmt.fmt.pix.height<<" pix:"<<fmt.fmt.pix.pixelformat<<endl;
	if (-1 == ioctl(fd, VIDIOC_S_FMT, &fmt))
	{
		cout<<"failed to set format"<<endl;
		exit(1);
	}

//attempt to read the video frame rate
	v4l2_frmivalenum std;
	CLEAR(std);
	std.index=0;
	std.pixel_format = fmt.fmt.pix.pixelformat;
	std.width = fmt.fmt.pix.width;
	std.height = fmt.fmt.pix.height;
	if (-1==ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &std))
	{
		cout<<"error reading video standard"<<endl;
	}
	if (std.type==V4L2_FRMIVAL_TYPE_DISCRETE)
	{
		
		cout<<"discrete"<<" rate:"<<std.discrete.denominator / std.discrete.numerator<<endl;
		std.index=1;
		do
		{
			if (-1==ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &std))
			{
				cout<<"error readign frame interval discrete"<<endl;
				break;
			}
		     	cout<<"discrete"<<" rate:"<<std.discrete.denominator / std.discrete.numerator<<endl;
			std.index++;
		}
		while(errno != EINVAL);
	}
	else if (std.type==V4L2_FRMIVAL_TYPE_STEPWISE)
		cout<<"stepwise"<<" rate:"<<std.stepwise.max.denominator / std.stepwise.max.numerator<<endl;
	else
		cout<<"type unknown, maybe it is continuous?"<<endl;

//read the video parameters
	v4l2_streamparm sp;
	CLEAR(sp);
	sp.type = fmt.type;
	if (-1==ioctl(fd, VIDIOC_G_PARM, &sp))
		cout<<"failed to read video parameters"<<endl;
	cout<<"***current frame rate: "<<sp.parm.capture.timeperframe.denominator<<" / "<<sp.parm.capture.timeperframe.numerator<<endl;

//set the vidoe parameters (frame rate to 30)
	sp.parm.capture.timeperframe.denominator = FRAME_RATE;
	if (-1==ioctl(fd, VIDIOC_S_PARM, &sp))
		cout<<"failed to set video parameters"<<endl;
	if (-1==ioctl(fd, VIDIOC_G_PARM, &sp))
		cout<<"failed to read video parameters"<<endl;
	cout<<"***current frame rate: "<<sp.parm.capture.timeperframe.denominator<<" / "<<sp.parm.capture.timeperframe.numerator<<endl;

//list out the extended controls
	struct v4l2_queryctrl qctrl;
	qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
	while (0 == ioctl(fd, VIDIOC_QUERYCTRL, &qctrl))
	{
		cout<<"E Con: "<<qctrl.name<<" id: "<<qctrl.id<<" type:"<<qctrl.type<<endl;
		qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
	}
	cout<<"done listing E Cons"<<endl;


//try setting exposure usign standard controls

	c.id = V4L2_CID_EXPOSURE_AUTO;
	c.value=V4L2_EXPOSURE_MANUAL;
	if (-1 == ioctl(fd, VIDIOC_S_CTRL, &c))
	{
		cout<<"failed to set exposure via standard controls"<<endl;
		switch(errno)
		{
		case EINVAL:
			cout<<"invalud id"<<endl;
			break;
		case ERANGE:
			cout<<"value out of range"<<endl;
			break;
		case EBUSY:
			cout<<"control is unavailable"<<endl;
			break;
		default:
			cout<<"unknown ioctl error: "<<errno<<endl;
		};
	}
//try to set the exposure value
	c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
	c.value = (int) 10000 / FRAME_RATE;
	if (-1 == ioctl(fd, VIDIOC_S_CTRL, &c))
	{
		cout<<"failed to set exposure value via standard controls"<<endl;
		switch(errno)
		{
		case EINVAL:
			cout<<"invalud id"<<endl;
			break;
		case ERANGE:
			cout<<"value out of range"<<endl;
			break;
		case EBUSY:
			cout<<"control is unavailable"<<endl;
			break;
		default:
			cout<<"unknown ioctl error: "<<errno<<endl;
		};
	}
	c.id = V4L2_CID_GAIN;
	c.value= GAIN;
	if (-1 == ioctl(fd, VIDIOC_S_CTRL, &c))
	{
		cout<<"failed to set gain via standard controls"<<endl;
		switch(errno)
		{
		case EINVAL:
			cout<<"invalud id"<<endl;
			break;
		case ERANGE:
			cout<<"value out of range"<<endl;
			break;
		case EBUSY:
			cout<<"control is unavailable"<<endl;
			break;
		default:
			cout<<"unknown ioctl error: "<<errno<<endl;
		};
	}


//now the format is set, so initialize the buffers
	cout<<"initializing buffers"<<endl;
	struct v4l2_requestbuffers req;
	req.count = BUFFERS_TO_REQUEST;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (-1==ioctl(fd, VIDIOC_REQBUFS, &req))
	{
		cout <<"failed to request buffers"<<endl;
		exit(1);
	}

	void * ptr;
	ptr = calloc(req.count, sizeof (*buffers));
	buffers = (buffer*)ptr;

	for (numBufs=0;numBufs<req.count;numBufs++)
	{
		struct v4l2_buffer buf;
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = numBufs;
		if (-1==ioctl(fd, VIDIOC_QUERYBUF, &buf))
		{
			cout <<"error querrying buffer num:"<<numBufs<<endl;
			exit(1);
		}
		buffers[numBufs].length = buf.length;
		buffers[numBufs].start = mmap(NULL, buf.length, 
						 PROT_READ | PROT_WRITE, 
						 MAP_SHARED, fd, buf.m.offset);
		if (MAP_FAILED == buffers[numBufs].start)
		{
			cout<<"mmap failed"<<endl;
			exit(1);
		}
	}
	cout<<"adding buffers to the queue"<<endl;
//now the buffers are ready, so add them to the queue
	for (int i=0;i<numBufs;i++)
	{
		struct v4l2_buffer buf;
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (-1==ioctl(fd, VIDIOC_QBUF, &buf))
		{
			cout<<"failled to queue buffer"<<endl;
			exit(1);
		}
	}


//enable streaming
	cout<<"enabling streaming"<<endl;
	enum v4l2_buf_type type;
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1==ioctl(fd, VIDIOC_STREAMON, &type))
		{
			cout<<"failed to enabled streaming"<<endl;
			exit(1);
		}

//set the vidoe parameters (frame rate to 30)
	sp.parm.capture.timeperframe.denominator = FRAME_RATE;
	if (-1==ioctl(fd, VIDIOC_S_PARM, &sp))
		cout<<"failed to set video parameters"<<endl;
	if (-1==ioctl(fd, VIDIOC_G_PARM, &sp))
		cout<<"failed to read video parameters"<<endl;
	cout<<"***current frame rate: "<<sp.parm.capture.timeperframe.denominator<<" / "<<sp.parm.capture.timeperframe.numerator<<endl;


		cout<<"Init succeeded"<<endl;




	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
        
	return;
}



void CaptureDevice::getV4LFrame()
{
	gettimeofday(&start, NULL);

        
//wait for a new frame
	struct timeval tv;

	tv.tv_sec = 2;
	tv.tv_usec = 0;


	if (checkedOutBuffer > -1)
	{
		if (-1 == ioctl(fd, VIDIOC_QBUF, &currBuf))
		{
			cout<<"failed to put the buffer back on the queue"<<endl;
			return;
		}
		checkedOutBuffer = -1;
	}

//	CLEAR(currBuf);
//	currBuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//	currBuf.memory = V4L2_MEMORY_MMAP;
//fetch the new frame
	if (-1 == ioctl(fd, VIDIOC_DQBUF, &currBuf))
	{
		cout<<"couldn't dequeue a frame. maybe none ready?"<<endl;
		return;
	}
	else
		checkedOutBuffer = currBuf.index;

//Convert the new frame to IplImage format
	V4LtoOpenCV();




	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);

	return;
}


void CaptureDevice::getVideoFrame()
{
	gettimeofday(&start, NULL);

        IplImage * tempImage = cvQueryFrame(camera); // camera is actually the video capture here
        //IplImage * tempImage2 = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
        //cvCvtColor(tempImage,tempImage2,CV_RGB2GRAY);
        cvResize(tempImage, frame, CV_INTER_LINEAR);
        //cout << tempImage->height << tempImage->width << endl;
        //cvCopy(tempImage, frame, NULL);
        //cvCvtColor(tempImage,frame,CV_RGB2GRAY);
        //cvReleaseImage(&tempImage);

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);



	return;
}


void CaptureDevice::closeDevice()
{
	gettimeofday(&start, NULL);


	if (fd!=-1)
		closeV4L();
	cvReleaseImage(&frame);


	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);

	return;
}

void CaptureDevice::closeV4L()
{
	gettimeofday(&start, NULL);


	close(fd);
	for (int i=0;i<numBufs;i++)
		if (-1==munmap(buffers[i].start, buffers[i].length))
		{
			cout<<"error unmmaping buffers"<<endl;
			exit(1);
		}




	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);

	return;
}

void CaptureDevice::V4LtoOpenCV()
{
	gettimeofday(&start, NULL);


	uchar* orig;
	uchar* dest;
	orig = (uchar *)buffers[currBuf.index].start;
	dest = (uchar *)frame->imageData;

	for (int y=0;y<PIC_HEIGHT;y++)
		for (int x=0;x<PIC_WIDTH;x++)
			dest[y*frame->widthStep+x] = orig[y*PIC_WIDTH*2 + x*2];




	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
        
	return;
}

void CaptureDevice::enumerate_menu()
{
	gettimeofday(&start, NULL);

        
	v4l2_querymenu querymenu;
	cout<<"  Menu items:"<<endl;
	CLEAR(querymenu);
	querymenu.id = queryctrl.id;
	for (querymenu.index=queryctrl.minimum;
	     querymenu.index<=queryctrl.maximum;
	     querymenu.index++)
	{
		if (0==ioctl(fd, VIDIOC_QUERYMENU, &querymenu))
		{
			cout<<"  "<<querymenu.name<<endl;
		}
		else
		{
			cout<<"error reading control menu"<<endl;
			exit(1);
		}
	}




	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
        
	return;
}



float CaptureDevice::getTime()
{
	return elapsedTime;
}

void CaptureDevice::resetTime()
{
    elapsedTime = 0.0;
}