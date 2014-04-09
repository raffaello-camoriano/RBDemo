
#include "detectNear.h"
#include <cv.h>

using namespace std;
using namespace cv;
using namespace yarp::sig;
using namespace yarp::os;

void DetectNear::loop()
{
	// Optionally
	// Read from Blob ports. 
	// Look for blobs bigger than a certain size or that fit some requisites
	// Reformat their coords, and send them to stereoDisparity via /stereoDisparity/box:i
	
	// Read 3D coords world image (whole image or blobs)
	ImageOf<PixelRgbFloat> *world  = worldInPort.read();  // read an stereo world image
    if(world == NULL)
        return;
	int height = world->height();
	int width = world->width();

	Mat worldImg((IplImage*) world->getIplImage());

	//Matrix wise computation
	//Mat dist(worldImg.rows,worldImg.cols,CV_64FC1);
	vector<Mat> channels(3);	
	split(worldImg, channels);		// split image into its channels
	//compute distance
	Mat dist = 


	float* pixelPtr = (float*)worldImg.data;
	int cn = worldImg.channels();
	for(int i = 0; i < worldImg.rows; i++)	{
		for(int j = 0; j < worldImg.cols; j += cn)		{
			// Extract coordinates
			float wx = pixelPtr[i*worldImg.cols*cn + j*cn + 0]; // X 
			float wy = pixelPtr[i*worldImg.cols*cn + j*cn + 1]; // Y
			float wz = pixelPtr[i*worldImg.cols*cn + j*cn + 2]; // Z
			//compute distance from reference origin
			float dist3D = 

		}
	}

	

	
	// compute all euclidean distances
	// Define elements as reachable or not depending on threshold
	// Find closer element
	
	


	im3DOutPort.write();
}

bool DetectNear::open()
{
    bool ret=true;
    ret = rpcPort.open("/detectNear/cmd:rpc");						// RPC port
    
	ret = ret && disparityPort.open("/detectNear/disp:i");			// port to receives disparity image
	ret = ret && worldInPort.open("/detectNear/world:i");			// Receive 3D coordinates of all points in the image
	ret = ret && blobsInPort.open("/detectNear/blobs:i");			// Receive info about the blobs in the image (optional)

	ret = ret && targetPort.open("/detectNear/target:o");			// 3D Coordinates of closest point.
	ret = ret && im3DOutPort.open("/detectNear/im3D:o");			// port to send out the 3D image with info drawn over  
   
    return ret;
}

bool DetectNear::close()
{
	
	printf("Closing");
    
	targetPort.setStrict();
	targetPort.write();

	im3DOutPort.setStrict();
	im3DOutPort.write();

    rpcPort.close();
	blobsInPort.close();
	disparityPort.close();
	worldInPort.close();
    targetPort.close();
    im3DOutPort.close();

	printf("Closed");
    return true;
}

bool DetectNear::interrupt()
{
	blobsInPort.interrupt();
	worldInPort.interrupt();
	disparityPort.interrupt();
	targetPort.interrupt();
	im3DOutPort.interrupt();

    return true;
}