
#ifndef __BLOB3DINFO_H__
#define __BLOB3DINFO_H__


#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>

class DetectNear
{
	yarp::os::Port									rpcPort;						// rpc port (receive commands via rpc), returns bounding box of selected blob
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgbFloat> > worldInPort;// input Port with info of 3D world
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > disparityPort; 
	yarp::os::BufferedPort<yarp::os::Bottle>		blobsInPort;					// input Port with blobs bounding boxes
    yarp::os::BufferedPort<yarp::os::Bottle>		targetPort;				// Send coordinates of closest point.
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > im3DOutPort;	// output image Port with info drawn over  


public:

    DetectNear()
    {
        // constructor
    }

    bool open();
	bool close();
	void loop(); 
	bool interrupt();
	
};

#endif
