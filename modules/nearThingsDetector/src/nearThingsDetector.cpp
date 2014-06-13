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
    detector->interrupt();
    return true;
}

/**********************************************************/
bool NearDetectorModule::close()
{
    rpcInPort.close();
    fprintf(stdout, "starting the shutdown procedure\n");   
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

    }else if (receivedCmd == "range"){
        bool ok = detector->setRange(command.get(1).asDouble());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Graspable Range not set. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "thresh"){
        bool ok = detector->setThresh(command.get(1).asInt());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Threshold for disparity considered set. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "confidence"){
        bool ok = detector->setConfMin(command.get(1).asDouble());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Confidence value given out of limits [0-1]. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "verbose"){
        bool ok = detector->setVerbose(command.get(1).asString());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "help"){
        reply.addVocab(Vocab::encode("many"));
        responseCode = Vocab::encode("ack");
        reply.addString("Available commands are:");
        reply.addString("origin (int) (int) (int)- set the coordinates from where the 3D distance is computed.");        
        reply.addString("range (double) - modifies the distance within which blobs are considered reachable.");
        reply.addString("thresh (int) - to sets the lower limit of disparity in terms of luminosity (0-255) that is considered. In other words, objects with luminosity under T, i.e. further away, wont be considered.");
        reply.addString("confidence (double) - Sets the confidence value [0-1] over which the obtained coordinates are sent.");
        reply.addString("verbose ON/OFF - Sets active the printouts of the program, for debugging or visualization.");
        reply.addString("help - produces this help.");
        reply.addString("quit - closes the module.");
        
        reply.addVocab(responseCode);
        return true;
    } else if (receivedCmd == "quit"){
        responseCode = Vocab::encode("ack");
        reply.addVocab(responseCode);
        closing = true;
        return true;
    }
    
    reply.addString("Invalid command, type [help] for a list of accepted commands.");
    
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
    verbose = moduleRF->check("verbose", Value(false)).asBool();
    range = moduleRF->check("range", Value(0.5)).asDouble();
    backgroundThresh = moduleRF->check("backgroundThresh", Value(50)).asInt();		// threshold of intensity if the disparity image, under which info is ignored.
    confidenceMin = moduleRF->check("confidenceMin", Value(0.8)).asDouble();		
    cannyThresh = moduleRF->check("cannyThresh", Value(20)).asDouble();
    minBlobSize = moduleRF->check("minBlobSize", Value(400)).asInt();
    gaussSize = moduleRF->check("gaussSize", Value(5)).asInt();
    dispThreshRatioLow = moduleRF->check("dispThreshRatioLow", Value(10)).asInt();    
    dispThreshRatioHigh = moduleRF->check("dispThreshRatioHigh", Value(20)).asInt();
    fixedRange = moduleRF->check("fixedRange", Value(true)).asBool();

    // XXX Move to ini file
    observableSpace.minX=-2; observableSpace.maxX=-0.05;
    observableSpace.minY=-0.5; observableSpace.maxY=0.5;
    observableSpace.minZ=-0.3; observableSpace.maxZ=0.3;
        
    bool ret=true;
    //create all ports
    fprintf(stdout,"Opening ports\n");

    /* Inputs ports */
    
    // worldInPortName = "/" + moduleName + "/world:i";
    // worldInPort.open(worldInPortName);

    dispInPortName = "/" + moduleName + "/disp:i";
    BufferedPort<ImageOf<PixelBgr>  >::open( dispInPortName.c_str() );    

    imLeftInPortName = "/" + moduleName + "/imLeft:i";
    imagePortInLeft.open(imLeftInPortName);
    
    imRightInPortName = "/" + moduleName + "/imRight:i";
    imagePortInRight.open(imRightInPortName);

    /* Output ports */

    imageOutPortName = "/" + moduleName + "/img:o";
    imageOutPort.open(imageOutPortName);

    imgBinOutPortName = "/" + moduleName + "/imgBin:o";
    imgBinOutPort.open(imgBinOutPortName);
    
    targetOutPortName = "/" + moduleName + "/target:o";
    targetOutPort.open(targetOutPortName);


    // Open disparity Thread
    string configFileDisparity = moduleRF->check("ConfigDisparity",Value("icubEyes.ini")).asString().c_str();
    string cameraContext = moduleRF->check("CameraContext",Value("cameraCalibration")).asString().c_str();
    string name = moduleRF->check("name",Value("blobCoordsExtractor")).asString().c_str();

    ResourceFinder cameraFinder;
    cameraFinder.setDefaultContext(cameraContext.c_str());
    cameraFinder.setDefaultConfigFile(configFileDisparity.c_str());
    cameraFinder.setVerbose();
    cameraFinder.configure(0,NULL);

    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local","/client/gaze");
 
    if (!clientGaze.open(option))
            return false;
    
    dispThr = new DisparityThread(name,cameraFinder, false, false, true); 
    dispThr->start();
    
    return ret;
}

/**********************************************************/
void NearThingsDetector::close()
{
    fprintf(stdout,"now closing ports...\n");
    
    // worldInPort.close();
    imageOutPort.close();
    imgBinOutPort.close();
    targetOutPort.close();
    BufferedPort<ImageOf<PixelBgr>  >::close();

    clientGaze.close();

    dispThr->stop();
    delete dispThr;
    
    fprintf(stdout,"finished closing input and output ports...\n");
}

/**********************************************************/
void NearThingsDetector::interrupt()
{	
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
   
    // worldInPort.interrupt();
    imageOutPort.interrupt();
    imgBinOutPort.interrupt();
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

bool NearThingsDetector::setThresh(int t)
{
    if ((t<0) ||(t>255)) {
        fprintf(stdout,"Please select a valid luminance value (0-255). \n");
        return false;
    }
    fprintf(stdout,"New Threshold is : %d\n", t);
    this->backgroundThresh = t;
    return true;
}

bool NearThingsDetector::setVerbose(string verb)
{
    if (verb == "ON"){
        verbose = true;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    } else if (verb == "OFF"){
        verbose = false;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    }    
    return false;
}

bool NearThingsDetector::setConfMin(float confid)
{
    if ((confid<0) ||(confid>1)) {
        fprintf(stdout,"Please select a valid confidence value [0-1]. \n");
        return false;
    }
    fprintf(stdout,"New minimum confidence Value : %f\n", confid);
    this->confidenceMin = confid;
    return true;
}


bool NearThingsDetector::observableTarget(const Scalar target_pos){
    if( (target_pos[0] < observableSpace.minX) || (target_pos[0] > observableSpace.maxX) ||
        (target_pos[1] < observableSpace.minY) || (target_pos[1] > observableSpace.maxY) ||
        (target_pos[2] < observableSpace.minZ) || (target_pos[2] > observableSpace.maxZ))
        {
        if(verbose){printf("Warning: target %f %f %f is outside of observable area.\n",target_pos[0],target_pos[1],target_pos[2]);}
        return false;          
    }
    else{
            return true;
    }
}


/**********************************************************/
void NearThingsDetector::onRead(ImageOf<PixelBgr> &disparity)
{
    yarp::os::Stamp ts;
    
    mutex.wait();    
    if(verbose){
        cout << endl;
        cout << "================ LOOP =================== "<< endl;}
    Scalar blue = Scalar(255,0,0);
    Scalar green = Scalar(0,255,0);
    Scalar red = Scalar(0,0,255);
    Scalar white = Scalar(255,255,255);

	/* Format disparty data to Mat grayscale */
    Mat disp((IplImage*) disparity.getIplImage());			
    cvtColor(disp, disp, CV_BGR2GRAY);						// Brg to grayscale

    /* Read camera Images to feed disp Thread */
    ImageOf<PixelRgb> *imInLeft = imagePortInLeft.read();  // read an image
    Mat leftIm((IplImage*) imInLeft->getIplImage());	
    ImageOf<PixelRgb> *imInRight = imagePortInRight.read();  // read an image
    Mat rightIm((IplImage*) imInRight->getIplImage());
    dispThr->setImages(leftIm,rightIm);
    while(!dispThr->checkDone()){
        yarp::os::Time::delay(0.1);}

    /* Prepare output image for visualization */
    ImageOf<PixelBgr> &imageOut  = imageOutPort.prepare();
    imageOut = disparity;	
    Mat imOut((IplImage*)imageOut.getIplImage(),false);

    /* Prepare binary image to ouput closesrt blob */
	ImageOf<PixelMono> &imgBin = imgBinOutPort.prepare();		// prepare an output image
	imgBin.resize(disparity.width(), disparity.height());		// Initialize features image
	Mat imgBinMat((IplImage*)imgBin.getIplImage(),false);
    
    /* Prepare output target port */
    Bottle &target = targetOutPort.prepare();
    target.clear();

    /* Filter disparity image to reduce noise */
    //GaussianBlur(disp, disp, Size(gaussSize,gaussSize), 1.5, 1.5);
    Mat threshIm;
    threshold(disp, threshIm, backgroundThresh, 1, CV_THRESH_BINARY);			// First
    multiply(disp, threshIm, disp);	
    cvtColor(disp, imOut, CV_GRAY2BGR);						// Grayscale to BGR    
    
    /* Find closest valid blob */
    double minVal, maxVal; 
    Point minLoc, maxLoc;	
    int fillRange = FLOODFILL_FIXED_RANGE;
    if (!fixedRange){
        fillRange = 4;}
    int fillSize = 0;	
    Mat aux = disp.clone();
    Mat fillMask = Mat::zeros(disp.rows + 2, disp.cols + 2, CV_8U);
    while (fillSize < minBlobSize){			
        minMaxLoc( aux, &minVal, &maxVal, &minLoc, &maxLoc );		// Look for brighter (closest) point
        fillSize = floodFill(aux, maxLoc, 0, 0, Scalar(maxVal/dispThreshRatioLow), Scalar(maxVal/dispThreshRatioHigh), fillRange);	// If its too small, paint it black and search again
    }

    floodFill(disp, fillMask, maxLoc, 255, 0, Scalar(maxVal/dispThreshRatioLow), Scalar(maxVal/dispThreshRatioHigh), FLOODFILL_MASK_ONLY + fillRange);	// Paint closest valid blob white
    threshold(disp, disp, 250, 255, CV_THRESH_BINARY);
    
    /* Find Contours */
    Mat edges;	
    vector<vector<Point > > contours, contours_aux;
    vector<Vec4i> hierarchy;
    //Canny( disp, edges, cannyThresh, cannyThresh*3, 3 );			// Detect edges using canny	
    findContours( fillMask(Range(1,disp.rows),Range(1,disp.cols)), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	
    /* If any blob is found */
    if (contours.size()>0){
        /* Double check that only the bigger blob is selected as the valid one*/
        int blobI = 0;
        for( int c = 0; c < contours.size(); c++ ){
			double a = contourArea(contours[c]);	    					// Find the area of contour
			if(a > minBlobSize){											// Keep only the bigger
                blobI = c;
            }
        }
        /* Mark closest blob for visualization*/        
        drawContours(imOut, contours, blobI, white, 2, 8); 
        drawContours(imgBinMat, contours, blobI, white, 2, 8); 

        /* check and plt the ROI returned by world*/
        Rect blobBox = boundingRect(contours[blobI]);
        if(verbose)
            {cout << " blob Box is  ["<< blobBox.tl().x << "," << blobBox.tl().y << "]:["<<blobBox.br().x << ","<<blobBox.br().y << "]"<< endl;}
        rectangle(imOut, blobBox, blue, 2 );

        /* Query stereoVision Library for the 3D coords of the point in the blob, filter the non-valid and get the average */
        Mat worldCoords(disp.size(), CV_32FC3, Scalar(0,0,0));  // Prepare canvas of whole image size
        vector<Mat> channels(3);	
        split(worldCoords-origin, channels);                    // split image into its channels X Y Z
        Bottle cmdSFM, responseSFM;
        for(int y = blobBox.y; y < blobBox.y + blobBox.height; y++) {
            for(int x = blobBox.x; x < blobBox.x + blobBox.width; x++) {                    
                // Query the StereoVision Library and to get the 3D coords of the point 
                Point2f point2D(x,y);
                Point3f point3D;
                dispThr->triangulate(point2D,point3D);  // XXX check if the coords on disparity image match the coord on wherever the library is computing the 3D values from
                // Read the 3D coords and compute the distance to the set reference frame origin       
                channels[0].at<float>(y,x) = point3D.x; // Get the X coordinate 
                channels[1].at<float>(y,x) = point3D.y; // Get the Y coordinate 
                channels[2].at<float>(y,x) = point3D.z; // Get the Z coordinate                 
            }              
        }
        merge(channels,worldCoords);    // Put coordinates back together in a single matrix for further procesisng
        
        /* Compute euclidean distance of all points matrixwise as sqrt(ch0^2+ch1^2+ch2^2). */        
        Mat dist3D;
        sqrt(channels[0].mul(channels[0])+channels[1].mul(channels[1])+channels[2].mul(channels[2]), dist3D); 

        /* Create a mask to ignore all points with non valid information (zero distance or out of contour). */
        Mat cntMask(disp.size(), CV_8UC1, Scalar(0));                   // do a mask by using drawContours (-1) on another black image
        drawContours( cntMask, contours, -1, Scalar(1), CV_FILLED, 8);  // Mask ignoring points outside the contours
        Mat zeroMask(disp.size(), CV_8UC1, Scalar(0));                  // Filter the black pixels from the image as another mask using cv::threshold with THRESH_BINARY and a threshold value of zero
        threshold(dist3D, zeroMask, 0, 1, 0);                           // Create a mask to ignore 0 values
        Mat mask(disp.size(), CV_8UC1, Scalar(0));                      // Perform an AND operation on the two masks and the original
        multiply(cntMask, zeroMask, mask, 1, CV_8UC1);                  // Compute jont mask: pixels within contours and with valid (non-zero) distances 
             
        Scalar center3DCoords = mean(worldCoords, mask);			    // Accpount only for coords where worldCoords data is valid  

        /* Get a measure of the confidence on the computation of the coordinates as the ratio between the number of valid points and the whole area of the blob*/
        int numValidPoints = countNonZero(mask);
        int numPoints =  contourArea(contours[blobI]);
        float confidence = (float)numValidPoints/ (float)numPoints;

        /* Get and draw centroid of the closest blob */ 
        Point center2DCoords;
        Moments mu = moments( contours[blobI], false );		
        center2DCoords = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        circle( imOut, center2DCoords, 4, red, -1, 8, 0 );
        std::ostringstream coordsStrX; 
        std::ostringstream coordsStrY;
        std::ostringstream coordsStrZ;
        coordsStrX << center3DCoords[0]; coordsStrX.precision(4);
        coordsStrY << center3DCoords[1]; coordsStrY.precision(4);
        coordsStrZ << center3DCoords[2]; coordsStrZ.precision(4);
        std::string coordsStr = "X: " + coordsStrX.str() +", Y:" + coordsStrY.str() + "Z:" + coordsStrZ.str();
        putText(imOut,coordsStr, Point2f(0,0), FONT_HERSHEY_COMPLEX, 0.6, white,2);

        /* Compute the average distance of each detected blob */        
        Mat reachness(disp.size(), CV_8UC3, Scalar(0,0,0));
        Scalar avgDist = mean(dist3D,mask);
        if(verbose)
            {cout << "Closest blob is at avg distance " << avgDist[0] << endl;}
        if (avgDist[0] > range){
            drawContours( reachness, contours, -1, red, CV_FILLED, 8);		// Paint red far blobs
        }else{
            drawContours( reachness, contours, -1, green, CV_FILLED, 8);		// Paint green close blobs
        }
        addWeighted( imOut, 0.7, reachness, 0.3, 0.0, imOut);

        /* Send Target coordinates*/
        if(verbose)
            {cout << "coords of the center of closest blob are : " << center3DCoords[0] << ", "<< center3DCoords[1]  << ", "<< center3DCoords[2] << endl;}

        if (observableTarget(center3DCoords) && confidence>confidenceMin){
            target.addDouble(center3DCoords[0]);
            target.addDouble(center3DCoords[1]);
            target.addDouble(center3DCoords[2]);  
            }
    }
    
    mutex.post();

    /* write info on output ports */
    imageOutPort.setEnvelope(ts);
    imageOutPort.write();
    imgBinOutPort.write();
    targetOutPort.write();

}
//empty line to make gcc happy

