/*
 * Copyright (C) 2014 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email:  tanis.mar@iit.it
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

#include "nearThingsDetector.h"

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/**********************************************************/
bool NearDetectorModule::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("nearThingsDetector"), "module name (string)").asString();

    setName(moduleName.c_str());

    handlerPortName =  "/";
    handlerPortName += getName();
    handlerPortName +=  "/rpc:i";
    if (!rpcInPort.open(handlerPortName.c_str()))    {
        fprintf(stdout, "%s : Unable to open input RPC port %s\n", getName().c_str(), handlerPortName.c_str());
        return false;
    }

    attach(rpcInPort);
    closing = false;

    /* create the thread and pass pointers to the module parameters */
    detector = new NearThingsDetector( moduleName, rf );

    /* now start the thread to do the work */
    detector->open();
    return true ;
}

/**********************************************************/
bool NearDetectorModule::interruptModule()
{
    closing = true;
    rpcInPort.interrupt();
    return true;
}

/**********************************************************/
bool NearDetectorModule::close()
{
    rpcInPort.close();
    fprintf(stdout, "starting the shutdown procedure\n");
    detector->interrupt();
    detector->close();
    fprintf(stdout, "deleting thread\n");
    delete detector;
    fprintf(stdout, "done deleting thread\n");
    return true;
}

/**********************************************************/
bool NearDetectorModule::updateModule()
{
    return !closing;
}

/**********************************************************/
bool NearDetectorModule::respond(const Bottle &command, Bottle &reply)
{
    /* This method is called when a command string is sent via RPC */
    reply.clear();  // Clear reply bottle

    /* Get command string */
    string receivedCmd = command.get(0).asString().c_str();
    int responseCode;   //Will contain Vocab-encoded response
    if (receivedCmd == "origin"){
        if (command.size() == 4){
            vector<double> origCoords(3);
            for (int i=1; i<=3; i++){
                origCoords[i-1] = command.get(i).asDouble();            
            }
            bool ok = detector->setOrigin(origCoords);
            if (ok)
                responseCode = Vocab::encode("ack");
            else {
                fprintf(stdout,"Coordinates format not accepted. Try [origin X Y Z]. \n");
                responseCode = Vocab::encode("nack");
            }
        }else
            responseCode = Vocab::encode("nack");
        reply.addVocab(responseCode);
        return true;
    }
    else if (receivedCmd == "range"){
        bool ok = detector->setRange(command.get(1).asDouble());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Threshold not set. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;
    } else if (receivedCmd == "quit"){
        closing =true;
    }

    fprintf(stdout,"Code not accepted. \n");
    responseCode = Vocab::encode("nack");
    reply.addVocab(responseCode);

    return true;
}

/**********************************************************/
double NearDetectorModule::getPeriod()
{
    return 0.1;
}

/**********************************************************/
NearThingsDetector::~NearThingsDetector()
{
    
}

/**********************************************************/
NearThingsDetector::NearThingsDetector( const string &moduleName, ResourceFinder &rf)
{
    fprintf(stdout,"initialising Variables\n");
    this->moduleName = moduleName;
    this->moduleRF = &rf;
}

/**********************************************************/
bool NearThingsDetector::open()
{
    this->useCallback();
    fprintf(stdout,"Parsing parameters\n");	

    Bottle originFrame = moduleRF->findGroup("origin");    
    if (originFrame.size()>0){
        for (int i=0; i<originFrame.size(); i++)
            origin[i] = originFrame.get(i+1).asDouble();
    }else{
        origin = Scalar(0,0,0);
    }
    fprintf(stdout,"Frame used : [%.2f,%.2f,%.2f]\n", origin[0], origin[1], origin[2] );	
    range = moduleRF->check("range", Value(0.5)).asDouble();
    backgroundThresh = moduleRF->check("backgroundThresh", Value(50)).asInt();		// threshold of intensity if the disparity image, under which info is ignored.
    cannyThresh = moduleRF->check("cannyThresh", Value(20)).asDouble();
    minBlobSize = moduleRF->check("minBlobSize", Value(400)).asInt();
    gaussSize = moduleRF->check("gaussSize", Value(5)).asInt();
    dispThreshRatioLow = moduleRF->check("dispThreshRatioLow", Value(10)).asInt();    
    dispThreshRatioHigh = moduleRF->check("dispThreshRatioHigh", Value(20)).asInt();
    
    bool ret=true;
    //create all ports
    fprintf(stdout,"Opening ports\n");

    dispInPortName = "/" + moduleName + "/disp:i";
    BufferedPort<ImageOf<PixelBgr>  >::open( dispInPortName.c_str() );
    
    //worldInPortName = "/" + moduleName + "/world:i";
    //worldInPort.open(worldInPortName);
    
    imageOutPortName = "/" + moduleName + "/img:o";
    imageOutPort.open(imageOutPortName);
    
    targetOutPortName = "/" + moduleName + "/target:o";
    targetOutPort.open(targetOutPortName);

    clientPortName = "/" + moduleName + "/rpc:o";
    rpcOutPort.open(clientPortName);
    
    return ret;
}

/**********************************************************/
void NearThingsDetector::close()
{
    fprintf(stdout,"now closing ports...\n");
    
    rpcOutPort.close();
    //worldInPort.close();
    imageOutPort.close();
    targetOutPort.close();
    BufferedPort<ImageOf<PixelBgr>  >::close();
    
    fprintf(stdout,"finished closing input and output ports...\n");
}

/**********************************************************/
void NearThingsDetector::interrupt()
{
	
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
    rpcOutPort.interrupt();
    //worldInPort.interrupt();
    imageOutPort.interrupt();
    targetOutPort.interrupt();
    BufferedPort<ImageOf<PixelBgr>  >::interrupt();
    fprintf(stdout,"finished interrupt ports\n");
}

bool NearThingsDetector::setOrigin(vector<double> o)
{   
    if (o.size()!=3){
        fprintf(stdout,"Coordinates format not valid, origin not modified. \n");
        return false;
    }

    for (int i=0; i<o.size(); i++)
        this->origin[i] = o[i];
    fprintf(stdout,"New origin Coords are : [%.2f,%.2f,%.2f]\n", origin[0], origin[1], origin[2] );
    return true;
}

bool NearThingsDetector::setRange(double r)
{
    if (r<0.1){
        fprintf(stdout,"Range is too small. \n");
        return false;
    }
    fprintf(stdout,"New Range is : %.2f\n", r);
    this->range = r;
    return true;
}

/**********************************************************/
void NearThingsDetector::onRead(ImageOf<PixelBgr> &disparity)
{
    yarp::os::Stamp ts;
    
    mutex.wait();
    cout << endl;
    cout << "================ LOOP =================== "<< endl;
    Scalar blue = Scalar(255,0,0);
    Scalar green = Scalar(0,255,0);
    Scalar red = Scalar(0,0,255);   

	/* Format disparty data to Mat grayscale */
    Mat disp((IplImage*) disparity.getIplImage());			
    cvtColor(disp, disp, CV_BGR2GRAY);						// Brg to grayscale
	
    /* Read 3D coords world image */
    /*
    ImageOf<PixelRgbFloat> *world  = worldInPort.read();	// read stereo world data
    if(world == NULL)
        return;
    Mat worldCoords((IplImage*) world->getIplImage());			// Reformat to Mat
    */

    /* Prepare output image for visualization */
    ImageOf<PixelBgr> &imageOut  = imageOutPort.prepare();
    imageOut = disparity;	
    Mat imOut((IplImage*)imageOut.getIplImage(),false);

    /* Prepare output target port */
    Bottle &target = targetOutPort.prepare();
    target.clear();

    /* Filter disparity image to reduce noise */
    GaussianBlur(disp, disp, Size(gaussSize,gaussSize), 1.5, 1.5);
    Mat threshIm;
    threshold(disp, threshIm, backgroundThresh, 1, CV_THRESH_BINARY);			// First
    multiply(disp, threshIm, disp);	
    //erode(disp,disp,Mat());
    //dilate(disp,disp, Mat());
    cvtColor(disp, imOut, CV_GRAY2BGR);						// Grayscale to BGR
    
    
    /* Find closest valid blob */
    double minVal, maxVal; 
    Point minLoc, maxLoc;	
    int fillSize = 0;	
    Mat aux = disp.clone();
    Mat fillMask = Mat::zeros(disp.rows + 2, disp.cols + 2, CV_8U);
    while (fillSize < minBlobSize){			
        minMaxLoc( aux, &minVal, &maxVal, &minLoc, &maxLoc );		// Look for brighter (closest) blob
        fillSize = floodFill(aux, maxLoc, 0, 0, Scalar(maxVal/dispThreshRatioLow), Scalar(maxVal/dispThreshRatioHigh),FLOODFILL_FIXED_RANGE);	// If its too small, paint it black and search again
    }

    floodFill(disp, fillMask, maxLoc, 255, 0, Scalar(maxVal/dispThreshRatioLow), Scalar(maxVal/dispThreshRatioHigh), FLOODFILL_MASK_ONLY + FLOODFILL_FIXED_RANGE);	// Paint closest valid blob white
    threshold(disp, disp, 240, 255, CV_THRESH_BINARY);
    
    /* Find Contours */
    Mat edges;	
    vector<vector<Point > > contours, contours_aux;
    vector<Vec4i> hierarchy;
    //Canny( disp, edges, cannyThresh, cannyThresh*3, 3 );			// Detect edges using canny	
    findContours( fillMask(Range(1,disp.rows),Range(1,disp.cols)), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	cout << "Found  " << contours.size() << " blobs." << endl;

    /* If any blob is found */
    if (contours.size()>0){
        printf("Checking there is only 1 blob:\n");
        /* Double check that only the bigger blob is selected as the valid one*/
        int blobI = 0;
        for( int c = 0; c < contours.size(); c++ ){
			double a = contourArea(contours[c]);	    					// Find the area of contour
			if(a > minBlobSize){											// Keep only the bigger
                blobI = c;
            }
        }
        printf("drawing blob contour:\n");
        /* Mark closest blob for visualization*/
        drawContours( imOut, contours, -1, blue, 2, 8); 
        
        /* Compute euclidean distance of all points matrixwise as sqrt(ch0^2+ch1^2+ch2^2). */
        /*
        Mat dist3D;
        vector<Mat> channels(3);	
        split(worldCoords-origin, channels);						// split image into its channels X Y Z
        sqrt(channels[0].mul(channels[0])+channels[1].mul(channels[1])+channels[2].mul(channels[2]), dist3D);	
        */
        
        /* Get and draw centroid of the closest blob */ 
        printf("Painting Center: \n");
        Point center;
        Moments mu = moments( contours[blobI], false );		
        center = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        circle( imOut, center, 4, red, -1, 8, 0 );
        
        /* Get and return valid 3D coords of the blob */
        printf("querying SFM: \n");
        Bottle cmdSFM, responseSFM;
        Scalar pointCoords;
        RNG rng;
        int validVals = 0;
        double dist3D;
        int cnt = 0;
        while (validVals == 0){
            /* Query the SFM module and to get the 3D coords of the point */
            cmdSFM.clear();
            responseSFM.clear();
            cmdSFM.addString("Point");
            cmdSFM.addInt(center.x);
            cmdSFM.addInt(center.y);
            rpcOutPort.write(cmdSFM, responseSFM); 
            printf("Got response: %s\n", responseSFM.toString().c_str());
                    
            /* Read the 3D coords and compute the distance to the set reference frame origin*/
            if (responseSFM.size() == 3){            
                for (int p=0; p<=2; p++){
                    pointCoords[p] = responseSFM.get(p).asDouble();            
                }
            }
            validVals = countNonZero(pointCoords);                      // Check if the data is Valid
            if (validVals == 0){                                        // If the data is not valid, update the query point
                center.x += rng.uniform(-1, 2);
                if (center.x<0) {center.x = 0;}
                if (center.x>disp.rows) {center.x = disp.rows;}
                center.y += rng.uniform(-1, 2);
                if (center.y<0) {center.y = 0;}
                if (center.y>disp.cols) {center.y = disp.cols;}
                printf("Looking for a better point, on coords: %d %d \n", center.x, center.y);
            }else{                                                      // If the data is valid, compute the distance
                pointCoords -= origin;                                                                         // subtract the offset            
                dist3D = sqrt(pointCoords[0]*pointCoords[0]+pointCoords[1]*pointCoords[1]+pointCoords[2]*pointCoords[2]);   // Compute euclidean distance
            }
          
        }
        
        //Scalar centerCoords = mean(worldCoords, mask);			    // Accpount only for coords where worldCoords data is valid  
        
        /* Create a mask to ignore all points with non valid information (zero distance or out of contour). *//*
        Mat cntMask(disp.size(), CV_8UC1, Scalar(0));					// do a mask by using drawContours (-1) on another black image
        drawContours( cntMask, contours, -1, Scalar(1), CV_FILLED, 8);	// Mask ignoring points outside the contours
        Mat zeroMask(disp.size(), CV_8UC1, Scalar(0));					// Filter the black pixels from the image as another mask using cv::threshold with THRESH_BINARY and a threshold value of zero
        threshold(dist3D, zeroMask, 0, 1, 0);							// Create a mask to ignore 0 values
        Mat mask(disp.size(), CV_8UC1, Scalar(0));						// Perform an AND operation on the two masks and the original
        multiply(cntMask, zeroMask, mask, 1, CV_8UC1);					// Compute jont mask: pixels within contours and with valid (non-zero) distances 
        */

        /* Compute the average distance of each detected blob */        
        Mat reachness(disp.size(), CV_8UC3, Scalar(0,0,0));
        //Scalar avgDist = mean(dist3D,mask);
        //cout << "Closest blob is at avg distance " << dist3D << endl;
        if (dist3D > range){
            drawContours( reachness, contours[blobI], -1, red, CV_FILLED, 8);		// Paint red far blobs
        }else{
            drawContours( reachness, contours[blobI], -1, green, CV_FILLED, 8);		// Paint green close blobs
        }
        addWeighted( imOut, 0.7, reachness, 0.3, 0.0, imOut);
      
        cout << "coords of the center of closest blob are : " << pointCoords[0] << ", "<< pointCoords[1]  << ", "<< pointCoords[2] << endl;
        target.addDouble(pointCoords[0]);
        target.addDouble(pointCoords[1]);
        target.addDouble(pointCoords[2]);  
    }
    
    mutex.post();

    /* write info on output ports */
    imageOutPort.setEnvelope(ts);
    imageOutPort.write();
    targetOutPort.write();
}
//empty line to make gcc happy
