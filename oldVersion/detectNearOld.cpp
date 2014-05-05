
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

	//XXXX To Dos:
	// - After computing and filtering the 3D distance, do contour detection
	// - Test on simulator putting a static object in front of the icub, and varying its distance
	// Send the following outputs:
	//  - XYZ of the closest point
	//  - Image with all the blobs within threshold shown, and the closest one with bounding box and centroid
	//  - Closer blob coords


	// XXX change method: 
	// Start with the disparity image
	// Get blobs from disparity
	// Using the world info, compute the average distance of the nonZero pixels inside each blob
	// Select blobs as reachable or not
	// Send out coordinates of the closest one. 

	
	// Read 3D coords world image
	cout << "read world data = "<< endl;
	ImageOf<PixelRgbFloat> *world  = worldInPort.read();  // read an stereo world image
    if(world == NULL)
        return;
	Mat worldImg((IplImage*) world->getIplImage());			// Transform world data into Mat
	
	cout << "prepare output ports "<< endl;

	//prepare an output test image
	ImageOf<PixelRgbFloat> &imTest = testImPort.prepare(); 
	imTest = *world;
	//Mat testIm((IplImage*)world->getIplImage(),false);
	// cout << "generate fake world data = "<< endl;
	//Mat worldImg(world->width(), world->height(), CV_32FC3,Scalar(3,3,3));//do a fake img all ones to test the distance computation
	//worldImg.copyTo(testIm);

	//Matrix wise computation of the Distance
	ImageOf<PixelFloat> &imageOut = im3DOutPort.prepare();		// prepare an output image
	imageOut.resize(world->width(), world->height());			// Initialize features image
	Mat dist3D((IplImage*)imageOut.getIplImage(),false);
	
	///*	
	//double timerMat = yarp::os::Time::now();
	vector<Mat> channels(3);	
	split(worldImg, channels);									// split image into its channels	
	//compute euclidean  distance of all points matrixwise as sqrt(ch0^2+ch1^2+ch2^2). 
	sqrt(channels[0].mul(channels[0])+channels[1].mul(channels[1])+channels[2].mul(channels[2]), dist3D);
	//cout << "dist3D = "<< endl << " "  << dist3D << endl << endl;
	//double timeMat = timerMat - yarp::os::Time::now();

	/*
	//Elementwise computation
	//double timerElem = yarp::os::Time::now();
	float* pixelPtr = (float*)worldImg.data;
	int cn = worldImg.channels();
	for(int i = 0; i < worldImg.rows; i++)	{
		for(int j = 0; j < worldImg.cols; j += cn)		{
			// Extract coordinates
			float wx = pixelPtr[i*worldImg.cols*cn + j*cn + 0]; // X 
			float wy = pixelPtr[i*worldImg.cols*cn + j*cn + 1]; // Y
			float wz = pixelPtr[i*worldImg.cols*cn + j*cn + 2]; // Z
			//compute distance from reference origin
			float pixDist = std::sqrt(pow(wx,2)+pow(wx,2)+pow(wx,2));
			dist3D.at<float>(i,j) = pixDist;
		}
	}
	*/
	//double timeElem = timerElem - yarp::os::Time::now();


	// Filter image to reduce noise
	GaussianBlur(dist3D, dist3D, Size(3,3), 1.5, 1.5);	
	erode(dist3D,dist3D,Mat());
	dilate(dist3D,dist3D, Mat());

	Point p_min, p_max;
	std::vector<cv::Point> nonZeros;   
	std::vector<cv::Point> tooFar;  

	//float minDist = findMin(dist3D, &p_min, &nonZeros);
	//float DetectNear::findMin(const cv::Mat& inputMat, const float thresh, std::vector<cv::Point> *tooFar,  cv::Point *min_i,  std::vector<cv::Point> *nonZeros)

	float minDist = 1000;
    const int M = dist3D.rows;
    const int N = dist3D.cols;
	Mat mask(dist3D);
	for (int m = 0; m < M; ++m) {
        const float* bin_ptr = dist3D.ptr<float>(m);
        for (int n = 0; n < N; ++n) {
            if (bin_ptr[n] > 0){								// save and compute only in non-zero values				
				nonZeros.push_back(Point(n,m));
				// Define elements as reachable or not depending on threshold
				if (bin_ptr[n] > thresh){						// check if things are too far away to be reachable				
					tooFar.push_back(Point(n,m));
					mask.at<float>(Point(n,m)) = 0;				// If a pixel is too far mask it as black
				} else{
					mask.at<float>(Point(n,m)) = 1;				// otherwise mark it white
				}	
				// Find closer element
				//cout << "Nonzero Val : " << bin_ptr[n] << " at " << Point(n,m) << endl;
				if (bin_ptr[n] < minDist){
					minDist = bin_ptr[n];
					p_min = Point(n,m);
					if (minDist<thresh){				//Check if the closest object is reachable
						closestReachable = true;}
					else{
						closestReachable = false;}
				}
			}
        }
    }

	//cv::imshow("mask",mask);
	//cv::imshow("dist3D",dist3D);

	dist3D.mul(mask);
	
	// XXX Multiply distance and mask to leave only close by pixels

	//Find Contours
	Mat edges;
	int cannyThresh = 100;
	int minBlobSize = 10;
	vector<vector<Point > > contours;
	vector<Vec4i> hierarchy;
	Canny( dist3D, edges, cannyThresh, cannyThresh*3, 3 );			// Detect edges using canny
	findContours( edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	printf(" Found %d contours\n", contours.size() );
	drawContours( worldImg, contours, -1, Scalar(128), 2, 8); 

	
	if (closestReachable == true){
		circle(worldImg, p_min, 10, Scalar(0,255,0), 1, 8, 0);}
	else{
		circle(worldImg, p_min, 10, Scalar(255,0,0), 1, 8, 0);}
	
    cout << "min: " << minDist << " at " << p_min << endl;


	// write info on output ports
	testImPort.write();
	im3DOutPort.write();
}

bool DetectNear::open(ResourceFinder &rf)
{
	thresh = 0.5;
	thresh = rf.check("range", Value(0.6)).asDouble();
    //threshold = rf.check("threshold", Value(10.0)).asDouble();
    //erode_itr = rf.check("erode_itr", Value(8)).asInt();
    //dilate_itr = rf.check("dilate_itr", Value(3)).asInt();
    //window_ratio = rf.check("window_ratio", Value(0.6)).asDouble();
    

    bool ret=true;
    ret = rpcPort.open("/detectNear/cmd:rpc");						// RPC port
    
	ret = ret && disparityPort.open("/detectNear/disp:i");			// port to receives disparity image
	ret = ret && worldInPort.open("/detectNear/world:i");			// Receive 3D coordinates of all points in the image
	ret = ret && blobsInPort.open("/detectNear/blobs:i");			// Receive info about the blobs in the image (optional)

	ret = ret && targetPort.open("/detectNear/target:o");			// 3D Coordinates of closest point.
	ret = ret && im3DOutPort.open("/detectNear/im3D:o");			// port to send out the 3D image with info drawn over  

	ret = ret && testImPort.open("/detectNear/testIm:o");
   
    return ret;
}

bool DetectNear::close()
{
	
	printf("Closing");
	testImPort.close();

    
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
	testImPort.interrupt();

	blobsInPort.interrupt();
	worldInPort.interrupt();
	disparityPort.interrupt();
	targetPort.interrupt();
	im3DOutPort.interrupt();

    return true;
}

//utils
/*
float DetectNear::findMin(const cv::Mat& inputMat, const float thresh, std::vector<cv::Point> *tooFar,  cv::Point *min_i,  std::vector<cv::Point> *nonZeros) {
	
	float minVal = 1000;
    assert(inputMat.cols > 0 && inputMat.rows > 0 && inputMat.channels() == 1 && inputMat.depth() == CV_32F);
    const int M = inputMat.rows;
    const int N = inputMat.cols;
    for (int m = 0; m < M; ++m) {
        const float* bin_ptr = inputMat.ptr<float>(m);
        for (int n = 0; n < N; ++n) {
            if (bin_ptr[n] > 0){						//save and compute only in non-zero values				
				nonZeros->push_back(Point(n,m));
				if (bin_ptr[n] > thresh){						//save and compute only in non-zero values				
					tooFar->push_back(Point(n,m));
				}
				//cout << "Nonzero Val : " << bin_ptr[n] << " at " << Point(n,m) << endl;
				if (bin_ptr[n] < minVal){
					minVal = bin_ptr[n];
					*min_i = Point(n,m);
				}
			}
        }
    }
	return minVal;
}
*/

