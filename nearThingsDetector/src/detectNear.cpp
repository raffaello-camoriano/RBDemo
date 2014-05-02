
#include "detectNear.h"
#include <cv.h>
#include "opencv2/opencv.hpp"
#include <stdlib.h>     //for using the function sleep

using namespace std;
using namespace cv;
using namespace yarp::sig;
using namespace yarp::os;

void DetectNear::loop()
{
	cout << "================ LOOP =================== "<< endl;
	
	// Read 3D coords world image
	//cout << "read world data = "<< endl;
	ImageOf<PixelRgbFloat> *world  = worldInPort.read();  // read an stereo world image
    if(world == NULL)
        return;
	Mat worldImg((IplImage*) world->getIplImage());
	//cout << "World data = " << endl << " " << worldImg << endl << endl;
	
	// Read disparity Data image	
	//cout << "read disparity data = "<< endl;
	ImageOf<PixelBgr> *disparity = disparityPort.read();       
    if(!disparity)
        return;
	Mat disp3((IplImage*) disparity->getIplImage());			// Transform disparity data into Mat
	Mat disp;
	cvtColor(disp3, disp, CV_BGR2GRAY);							// Create greyscale image for further analysis

	// prepare an output image
	cout << "Preparing output image... "<< endl;
	ImageOf<PixelBgr> &imageOut = im3DOutPort.prepare();		// prepare an output image
	imageOut = *disparity;	
	Mat imOut((IplImage*)imageOut.getIplImage(),false);

	//Prepare output target port    
	Bottle &target = targetOutPort.prepare();
    target.clear();

	// prepare a test output image
	cout << "Preparing test output image... "<< endl;
	ImageOf<PixelMono> &imTestOut = imTestOutPort.prepare();		// prepare an output TEST image
	imTestOut.resize(disparity->width(), disparity->height());		// Initialize features image
	imTestOut.zero();
	//Mat imTest = cvarrToMat(imTestOut.getIplImage(),false);
	Mat imTest((IplImage*)imTestOut.getIplImage(),false);

	// Filter disparity image to reduce noise 
	cout << "Filtering image to reduce noise... "<< endl;
	GaussianBlur(disp, disp, Size(5,5), 1.5, 1.5);
	Mat threshIm;
	threshold(disp, threshIm, thresh, 1, CV_THRESH_BINARY);			// Firs
	multiply(disp, threshIm, disp);	
	erode(disp,disp,Mat());
	dilate(disp,disp, Mat());
	
	// Find closest valid blob
	double minVal, maxVal; 
	Point minLoc, maxLoc;	
	int fillSize = 0;	
	Mat aux = disp.clone();
	while (fillSize < minBlobSize){			
		minMaxLoc( aux, &minVal, &maxVal, &minLoc, &maxLoc );		// Look for brighter (closest) blob
		fillSize = floodFill(aux, maxLoc, 0, 0, Scalar(maxVal/15), Scalar(maxVal/30));	// If its too small, paint it black and search again
	}
	floodFill(disp, maxLoc, 255, 0, Scalar(maxVal/15), Scalar(maxVal/30));	// Paint closest valid blob white
	threshold(disp, disp, 250, 255, CV_THRESH_BINARY);			// Keep only the white (closest) blob

	
	//Find Contours	
	Mat edges;	
	vector<vector<Point > > contours, contours_aux;
	vector<Vec4i> hierarchy;
	//Canny( disp, edges, cannyThresh, cannyThresh*3, 3 );			// Detect edges using canny	
	findContours( disp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	drawContours( imOut, contours, -1, Scalar(255,0, 0), 2, 8); 
	
	//Compute euclidean distance of all points matrixwise as sqrt(ch0^2+ch1^2+ch2^2). 
	cout << "Computing 3D distance from world data... "<< endl;
	Mat dist3D;
	vector<Mat> channels(3);	
	split(worldImg, channels);						// split image into its channels X Y Z
	sqrt(channels[0].mul(channels[0])+channels[1].mul(channels[1])+channels[2].mul(channels[2]), dist3D);	

	// Create a mask to ignore all points with non valid information (zero distance or out of contour).
	cout << "Creating mask to keep valid distance data... "<< endl;
	Mat cntMask(disp.size(), CV_8UC1, Scalar(0));					// do a mask by using drawContours (-1) on another black image
	//threshold(disp, cntMask, 250, 1, CV_THRESH_BINARY);	
	drawContours( cntMask, contours, -1, Scalar(1), CV_FILLED, 8);	// Mask ignoring points outside the contours
	Mat zeroMask(disp.size(), CV_8UC1, Scalar(0));					// Filter the black pixels from the image as another mask using cv::threshold with THRESH_BINARY and a threshold value of zero
	threshold(dist3D, zeroMask, 0, 1, 0);							// Create a mask to ignore 0 values
	Mat mask(disp.size(), CV_8UC1, Scalar(0));						// Perform an AND operation on the two masks and the original
	multiply(cntMask, zeroMask, mask, 1, CV_8UC1);					// Compute jont mask: pixels within contours and with valid (non-zero) distances 
	multiply(cntMask, zeroMask, imTest, 255, CV_8UC1);					// Compute jont mask: pixels within contours and with valid (non-zero) distances 

	// Compute the average distance of each detected blob
	//cout << "Computing avg distance of each blob... "<< endl;
	Mat reachness(disp.size(), CV_8UC3, Scalar(0,0,0));
	Scalar avgDist = mean(dist3D,mask);
	cout << "Closest blob is at avg distance " << avgDist[0] << endl << endl;
	if (avgDist[0]>range){
		drawContours( reachness, contours, -1, Scalar(0,0,255), CV_FILLED, 8);		// Paint red far blobs
	}else{
		drawContours( reachness, contours, -1, Scalar(0,255,0), CV_FILLED, 8);		// Paint green close blobs
	}	
	addWeighted( imOut, 0.7, reachness, 0.3, 0.0, imOut);

	// Draw centroid of the closest blob and return its 3D coords
	if (contours.size()>0){
		// Draw centroid
		Point center;
		Moments mu = moments( contours[0], false );		
		center = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
		circle( imOut, center, 4, Scalar(0,0,255), -1, 8, 0 );
		
		// Get valid 3D coords of the blob
		cout << "Getting 3D coords of center " << endl;
		//PixelRgbFloat centerCoords;// = world->operator()(center.x,center.y);
		Scalar centerCoords = mean(worldImg, mask);

		cout << "coords of the center of closest blob are : " << centerCoords[0] << ", "<< centerCoords[1]  << ", "<< centerCoords[2] << endl;
		//Bottle target;
		target.addDouble(centerCoords[0]);
		target.addDouble(centerCoords[1]);
		target.addDouble(centerCoords[2]);
	}

	// write info on output ports
	im3DOutPort.write();
	targetOutPort.write();
	imTestOutPort.write();
}

bool DetectNear::open(ResourceFinder &rf)
{
	range = 1;
	minBlobSize = 400;
	thresh = 50;				// threshold of intensity if the disparity image, under which info is ignored.
	cannyThresh = 20;
	//range = rf.check("range", Value(0.6)).asDouble();
    //thresh = rf.check("threshold", Value(10.0)).asDouble();
	//minBlobSize = rf.check("minBlobSize", Value(50)).asInt();
    //erode_itr = rf.check("erode_itr", Value(8)).asInt();
    //dilate_itr = rf.check("dilate_itr", Value(3)).asInt();
    //window_ratio = rf.check("window_ratio", Value(0.6)).asDouble();
    

    bool ret=true;
    ret = rpcPort.open("/detectNear/cmd:rpc");						// RPC port

	ret = ret && imTestOutPort.open("/detectNear/imTest:o");		///XXX Test port
    
	ret = ret && disparityPort.open("/detectNear/disp:i");			// port to receives disparity image
	ret = ret && worldInPort.open("/detectNear/world:i");			// Receive 3D coordinates of all points in the image
	ret = ret && blobsInPort.open("/detectNear/blobs:i");			// Receive info about the blobs in the image (optional)

	ret = ret && targetOutPort.open("/detectNear/target:o");			// 3D Coordinates of closest point.
	ret = ret && im3DOutPort.open("/detectNear/im3D:o");			// port to send out the 3D image with info drawn over  
	   
    return ret;
}

bool DetectNear::close()
{
	
	printf("Closing");

	imTestOutPort.close();				//XXX
    
	targetOutPort.setStrict();
	targetOutPort.write();

	im3DOutPort.setStrict();
	im3DOutPort.write();

    rpcPort.close();
	blobsInPort.close();
	disparityPort.close();
	worldInPort.close();
    targetOutPort.close();
    im3DOutPort.close();

	printf("Closed");
    return true;
}

bool DetectNear::interrupt()
{
	imTestOutPort.interrupt();			//XXX

	blobsInPort.interrupt();
	worldInPort.interrupt();
	disparityPort.interrupt();
	targetOutPort.interrupt();
	im3DOutPort.interrupt();

    return true;
}

