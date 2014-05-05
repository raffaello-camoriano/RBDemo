#ifndef __DETECTNEAR_H__
#define __DETECTNEAR_H__


#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>

#include <cv.h>

class DetectNear
{
	yarp::os::Port									rpcPort;							// rpc port (receive commands via rpc), returns bounding box of selected blob

	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imTestOutPort;		// output image Port with info drawn over  

	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgbFloat> > worldInPort;	// input Port with info of 3D world
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > disparityPort;	// REceives disparity greyscale image
	yarp::os::BufferedPort<yarp::os::Bottle>		blobsInPort;						// input Port with blobs bounding boxes
    yarp::os::BufferedPort<yarp::os::Bottle>		targetOutPort;							// Send coordinates of closest point.
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > im3DOutPort;		// output image Port with info drawn over  

	//yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgbFloat> >	testImPort;

	int backgroundThresh;
	int cannyThresh;
	int minBlobSize;

	int dispThreshRatioLow;
	int dispThreshRatioHigh;

	int gaussSize;
	float range;
	float minDist;
	bool closestReachable;	

public:

    DetectNear() 
    {
        // constructor
    }

    bool open(yarp::os::ResourceFinder &rf);
	bool close();
	void loop(); 
	bool interrupt();
	
};

#endif
