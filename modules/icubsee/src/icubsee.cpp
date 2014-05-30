/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Sriram Kumar
 * email: sriram.kishore@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
/*
An Example program to get image from the icub and classify the foreground from background integrated with opencv.

Copyright (C) 2010 RobotCub Consortium
 
Author: Sriram Kumar 

CopyPolicy: Released under the terms of the GNU GPL v2.0. 

Scenario : 
To write a simple yarp code that does the following things: 

(1) get images from one/two robot camera; 
(2) build an internal representation of the scene background (the scene is assumed to be nearly static); 
(3) stream out an image whose pixels are classified as either background or foreground.
*/

#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/all.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;
class icubsee: public RFModule {
	/* module parameters */

	string moduleName;
	string robotName;
	string LeftCam_PortName;
	string RightCam_PortName;
	string inputPortName_left;
	string inputPortName_right;
	string outputPortName_left;
	string outputPortName_right;
	string cameraConfigFilename;
	Port handlerPort_left;
	Port handlerPort_right;
	int thresholdValue;
	int i;
	PixelRgb rgbPixel;
	cv::Mat initial_l;
	cv::Mat initial_r;
	BufferedPort<ImageOf<PixelRgb> > imageIn_left;
	BufferedPort<ImageOf<PixelRgb> > imageIn_right;
	BufferedPort<ImageOf<PixelRgb> > imageOut_left;
	BufferedPort<ImageOf<PixelRgb> > imageOut_right;

public:

	bool configure(yarp::os::ResourceFinder &rf);
	bool interruptModule();
	bool close();
	double getPeriod();
	bool updateModule();

private:
	ImageOf<PixelRgb> Mattoyarp(cv::Mat mat, ImageOf<PixelRgb> &binary_image,
			bool color);
	Mat yarptoMat(ImageOf<PixelRgb> *image, bool formatChange_flag);
};

Mat icubsee::yarptoMat(ImageOf<PixelRgb> *image, bool formatChange_flag) {
	IplImage *cvImage = cvCreateImage(cvSize(image->width(), image->height()),
			IPL_DEPTH_8U, 1);
	if (formatChange_flag) {
		cvCvtColor((IplImage*) image->getIplImage(), cvImage, CV_RGB2GRAY);
	} else {
		cvImage = cvCloneImage((IplImage*) image->getIplImage());
	}
	return cvarrToMat(cvImage);
}

ImageOf<PixelRgb> icubsee::Mattoyarp(cv::Mat mat,
		ImageOf<PixelRgb> &binary_image, bool color) {
	int x, y;
	IplImage *ipl = new IplImage(mat);
	uchar *data = (uchar*) ipl->imageData;
	for (x = 0; x < ipl->height; x++) {
		for (y = 0; y < ipl->width; y++) {
			if (color) {
				int da = data[x * ipl->widthStep + y * ipl->nChannels];
				rgbPixel.r = (unsigned char) da;
				rgbPixel.g = (unsigned char) da;
				rgbPixel.b = (unsigned char) da;
			} else {
				rgbPixel.r = (unsigned char) data[x * ipl->widthStep
						+ y * ipl->nChannels + 0];
				rgbPixel.g = (unsigned char) data[x * ipl->widthStep
						+ y * ipl->nChannels + 1];
				rgbPixel.b = (unsigned char) data[x * ipl->widthStep
						+ y * ipl->nChannels + 2];
			}
			binary_image(y, x) = rgbPixel;
		}
	}
//	cvReleaseImage(&ipl);
	return binary_image;
}

bool icubsee::updateModule() {
	unsigned char value;
	if (i == 1) {
		initial_l = yarptoMat(imageIn_left.read(), true);
		initial_r = yarptoMat(imageIn_right.read(), true);
	}
	i = 0;
	ImageOf < PixelRgb > *image_left = imageIn_left.read();
	ImageOf < PixelRgb > *image_right = imageIn_right.read();
	if (image_left != NULL && image_right != NULL) {
		ImageOf < PixelRgb > &binary_image_l = imageOut_left.prepare();
		binary_image_l.resize(image_left->width(), image_left->height());
		ImageOf < PixelRgb > &binary_image_r = imageOut_right.prepare();
		binary_image_r.resize(image_right->width(), image_right->height());
		cv::Mat mat_left = yarptoMat(image_left, true);
		cv::Mat mat_right = yarptoMat(image_right, true);
		cv::Mat Original_left = yarptoMat(image_left, false);
		cv::Mat Original_right = yarptoMat(image_right, false);
		// Motion detection starts from here
		cv::absdiff(initial_l, mat_left, mat_left);
		cv::threshold(mat_left, mat_left, 15, 255, CV_THRESH_BINARY);
		cv::absdiff(initial_r, mat_right, mat_right);
		cv::threshold(mat_right, mat_right, 15, 255, CV_THRESH_BINARY);

		cv::erode(mat_left, mat_left, cv::Mat(), cv::Point(-1, -1), 1);
		cv::dilate(mat_left, mat_left, cv::Mat(), cv::Point(-1, -1), 1);
		cv::erode(mat_right, mat_right, cv::Mat(), cv::Point(-1, -1), 1);
		cv::dilate(mat_right, mat_right, cv::Mat(), cv::Point(-1, -1), 1);

		std::vector < std::vector<cv::Point> > contours;
		std::vector < cv::Point > points;
		cv::findContours(mat_left, contours, CV_RETR_LIST,
				CV_CHAIN_APPROX_NONE);
		for (size_t i = 0; i < contours.size(); i++) {
			for (size_t j = 0; j < contours[i].size(); j++) {
				cv::Point p = contours[i][j];
				points.push_back(p);
			}
		}

		if (points.size() > 0) {
			cv::Rect brect = cv::boundingRect(cv::Mat(points).reshape(2));
			cv::rectangle(Original_left, brect.tl(), brect.br(),
					cv::Scalar(100, 100, 200), 2, CV_AA);
		}
		cv::findContours(mat_right, contours, CV_RETR_LIST,
				CV_CHAIN_APPROX_NONE);
		for (size_t i = 0; i < contours.size(); i++) {
			for (size_t j = 0; j < contours[i].size(); j++) {
				cv::Point p = contours[i][j];
				points.push_back(p);
			}
		}
		if (points.size() > 0) {
			cv::Rect brect = cv::boundingRect(cv::Mat(points).reshape(2));
			cv::rectangle(Original_right, brect.tl(), brect.br(),
					cv::Scalar(100, 100, 200), 2, CV_AA);
		}

//		binary_image_l = Mattoyarp(Original_left, binary_image_l, false);
//		binary_image_r = Mattoyarp(Original_right, binary_image_r, false);
		binary_image_l = Mattoyarp(mat_left, binary_image_l, false);
		binary_image_r = Mattoyarp(mat_right, binary_image_r, false);

		imageOut_left.write();
		imageOut_right.write();
	}
	return true;
}

bool icubsee::configure(yarp::os::ResourceFinder &rf) {
	moduleName =
			rf.check("name", Value("icubsee"), "module name (string)").asString();
	setName(moduleName.c_str());
	robotName =
			rf.check("robot", Value("icubSim"), "Robot name (string)").asString();
	inputPortName_left = "/";
	inputPortName_left += rf.check("leftimage_port", Value("limg:i"),
			"Input image port left (string)").asString();
	inputPortName_right = "/";
	inputPortName_right += rf.check("rightimage_port", Value("rimg:i"),
			"Input image port right (string)").asString();
	outputPortName_left = "/";
	outputPortName_left += rf.check("leftcamera_view", Value("left:o"),
			"Output image port left (string)").asString();
	outputPortName_right = "/";
	outputPortName_right += rf.check("rightcamera_view", Value("right:o"),
			"Output image port right (string)").asString();
	if (!imageIn_left.open(inputPortName_left.c_str())) {
		cout << getName() << ": unable to open port " << inputPortName_left
				<< endl;
		return false;  // unable to open; let RFModule know so that it won't run
	}
	if (!imageIn_right.open(inputPortName_right.c_str())) {
		cout << getName() << ": unable to open port " << inputPortName_right
				<< endl;
		return false;  // unable to open; let RFModule know so that it won't run
	}
	if (!imageOut_left.open(outputPortName_left.c_str())) {
		cout << getName() << ": unable to open port " << outputPortName_left
				<< endl;
		return false;  // unable to open; let RFModule know so that it won't run
	}
	if (!imageOut_right.open(outputPortName_right.c_str())) {
		cout << getName() << ": unable to open port " << outputPortName_right
				<< endl;
		return false;  // unable to open; let RFModule know so that it won't run
	}
	thresholdValue =
			rf.check("threshold", Value(0.8), "Key value (int)").asInt();
	i = 1;
	return true;
}

bool icubsee::interruptModule() {
	imageIn_left.interrupt();
	imageOut_left.interrupt();
	imageIn_right.interrupt();
	imageOut_right.interrupt();
	return true;
}

double icubsee::getPeriod() {
	return 0.0;
}

bool icubsee::close() {
	imageIn_left.close();
	imageOut_left.close();
	imageIn_right.close();
	imageOut_right.close();
	return true;
}

int main(int argc, char * argv[]) {
	Network yarp;
	if (!yarp.checkNetwork()) {
		cout << "YARP server not available!" << endl;
		return -1;
	}
	icubsee icubse;
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("icubsee_configfile.ini"); //overridden by --from parameter
	rf.setDefaultContext("icubsee");   //overridden by --context parameter
	rf.configure(argc, argv);
	icubse.runModule(rf);

	return 0;
}
