
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
	//XXXX To Dos:
	// - After computing and filtering the 3D distance, do contour detection
	// - Test on simulator putting a static object in front of the icub, and varying its distance
	// Send the following outputs:
	//  - XYZ of the closest point
	//  - Image with all the blobs within threshold shown, and the closest one with bounding box and centroid
	//  - Closer blob coords


	// Accept blob coordinates as an input
	// in that case, edge and contour detection has to be done nonetheless.

	// XXX change method: 
	// Start with the disparity image
	// Get blobs from disparity
	// Using the world info, compute the average distance of the nonZero pixels inside each blob
	// Select blobs as reachable or not
	// Send out coordinates of the closest one. 

	
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

	// Read blobs
	Bottle *blobBoxes = blobsInPort.read();						// read the blobs bounding boxes
	// check we actually got blob information
	bool blobsGiven = false;
	vector<Rect> blobRects;
	if(blobBoxes->size() > 0){
		cout << " Blobs received"<< endl;
		blobsGiven = true;
		for (int b=0; b<blobBoxes->size(); b++){				// Read blob bounding boxes from bottles
			Point tl,br;
			Bottle *item = blobBoxes->get(b).asList();
			tl.x=(int)item->get(0).asInt();
			tl.y=(int)item->get(1).asInt();
			br.x=(int)item->get(2).asInt();
			br.y=(int)item->get(3).asInt();
			Rect blob(tl,br);
			blobRects.push_back(blob);
		}

	}else{
		Rect blob(0,0,disparity->width(),disparity->height());	// Select all image as blob	
		blobRects.push_back(blob);
	}

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
	imTestOut.resize(disparity->width(), disparity->height());	// Initialize features image
	imTestOut.zero();
	//Mat imTest = cvarrToMat(imTestOut.getIplImage(),false);
	Mat imTest((IplImage*)imTestOut.getIplImage(),false);

	// Filter disparity image to reduce noise 
	cout << "Filtering image to reduce noise... "<< endl;
	GaussianBlur(disp, disp, Size(5,5), 1.5, 1.5);
	//GaussianBlur(disp, imTest, Size(3,3), 1.5, 1.5);	
	Mat threshIm;
	threshold(disp,threshIm,thresh, 1, CV_THRESH_BINARY);
	multiply(disp,threshIm,disp);
	multiply(disp,threshIm,imTest);
	//disp.mul(threshIm);
	//threshold(disp,imTest,thresh, 255.0, CV_THRESH_BINARY);

	//equalizeHist(disp,disp);
	//erode(disp,disp,Mat());
	//dilate(disp,disp, Mat());
	

	//Find Contours	
	Mat edges;	
	vector<vector<Point > > contours;
	vector<Vec4i> hierarchy;
	Canny( disp, edges, cannyThresh, cannyThresh*3, 3 );			// Detect edges using canny	
	for( int i = 0; i < blobRects.size(); i++ ){
		rectangle(imOut,blobRects[i],Scalar(255,0,0));
		cout << "Finiding contours "<< endl;
		vector<vector<Point > > contours_aux;
		findContours( edges(blobRects[i]), contours_aux, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		cout << "Finding large blobs... "<< endl;
		for( int i = 0; i < contours_aux.size(); i++ ){
			double a=contourArea(contours_aux[i]);						// Find the area of contour
			if(a>minBlobSize){											// Keep only big contours
				contours.push_back(contours_aux[i]);					
			}
		}
	}
	printf(" Found %d large contours\n", contours.size() );
	drawContours( imOut, contours, -1, Scalar(255,255, 255), 2, 8); 

	//compute euclidean distance of all points matrixwise as sqrt(ch0^2+ch1^2+ch2^2). 
	cout << "Computing 3D distance from world data... "<< endl;
	Mat dist3D;
	vector<Mat> channels(3);	
	split(worldImg, channels);									// split image into its channels		
	sqrt(channels[0].mul(channels[0])+channels[1].mul(channels[1])+channels[2].mul(channels[2]), dist3D);	

	// Compute average distance of each contour/blob
	cout << "Creating masks... "<< endl;
	Mat cntMask(disp.size(), CV_8UC1, Scalar(0));
	cout << "Mask pixels outside contours... "<< endl;	// do a mask by using drawContours (-1) on another black image
	drawContours( cntMask, contours, -1, Scalar(1), CV_FILLED, 8);		// Within contours mask
	//cout << "M = " << endl << " " << cntMask << endl << endl;
	
	Mat zeroMask(disp.size(), CV_8UC1, Scalar(0));
	cout << "Mask zero value pixels ... "<< endl;						// Filter the black pixels from the image as another mask using cv::threshold with THRESH_BINARY and a threshold value of zero
	threshold(dist3D, zeroMask, 0, 1, 0);// Create a mask to ignore 0 values
	//cout << "M = " << endl << " " << zeroMask << endl << endl;
	
	Mat mask(disp.size(), CV_8UC1, Scalar(0));
	cout << "Add masks ... "<< endl;								// Perform an AND operation on the two masks and the original
	//bitwise_and(cntMask,zeroMask,mask);							// Compute jont mask: pixels within contours and with valid distances 
	multiply(cntMask, zeroMask, mask, 1, CV_8UC1);					// Compute jont mask: pixels within contours and with valid distances 
	//cout << "M = " << endl << " " << mask << endl << endl;
	
	//Mat validDist(disp.rows,disp.cols,CV_32F);
	//cout << "Get unmasked distance data... "<< endl;
	//multiply(mask,dist3D,validDist,1,CV_32F);
	//cout << "M = " << endl << " " << validDist << endl << endl;

	
	cout << "Computing avg distance of each blob... "<< endl;	
	Mat reachness(disp.size(), CV_8UC3, Scalar(0,0,0));
	minDist = 1000;
	for( int i = 0; i < contours.size(); i++ ){
		Rect boundRect = boundingRect(contours[i]);// get bounding box
		//rectangle(imOut,boundRect, Scalar(255, 0 ,0));
		//cout << "Blob" << i << "in rectangle" << boundRect.br() << ", "<< boundRect.tl() << "within image" << dist3D.size << endl;
		Scalar avgDist = mean(dist3D(boundRect),mask(boundRect));
		cout << "Blob " << i << " is at avg distance " << avgDist << endl << endl;
		char strI[4]; sprintf_s(strI,"%d",i);
		putText(imOut,strI,boundRect.br(), CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,255));
		// Paint it red if it is > range and green if it is closer		
		if (avgDist[0]>range){
			drawContours( reachness, contours, i, Scalar(0,0,255), CV_FILLED, 8);		// Paint red far blobs
		}else{
			drawContours( reachness, contours, i, Scalar(0,255,0), CV_FILLED, 8);		// Paint green close blobs
		}
		if (avgDist[0]<minDist){
			minDist = avgDist[0];
			closestBlobI = i;
			closestReachable =true;
		}
	}
	addWeighted( imOut, 0.7, reachness, 0.3, 0.0, imOut);

	// Draw bounding box and centroid of the closest blob
	if (contours.size()>0){
		Point center;
		Moments mu = moments( contours[closestBlobI], false );
		center = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
		circle( imOut, center, 4, Scalar(0,0,255), -1, 8, 0 );
	
		cout << "Getting 3D coords of center " << endl;
		PixelRgbFloat centerCoords = world->operator()(center.x,center.y);
		cout << "coords of the center of closest blob are : " << centerCoords.b<< ", "<< centerCoords.g << ", "<< centerCoords.b << endl;
		//Bottle target;
		target.addDouble(centerCoords.r);
		target.addDouble(centerCoords.g);
		target.addDouble(centerCoords.b);
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
	thresh = 50;
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
	//testImPort.close();

    
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
	//testImPort.interrupt();

	blobsInPort.interrupt();
	worldInPort.interrupt();
	disparityPort.interrupt();
	targetOutPort.interrupt();
	im3DOutPort.interrupt();

    return true;
}

