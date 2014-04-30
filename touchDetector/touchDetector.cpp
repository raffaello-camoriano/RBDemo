#include <stdio.h>
#include <iostream>
#include <string>

#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/dev/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>

#include <yarp/math/Math.h>

#include <gsl/gsl_math.h>

#include <iCub/skinDynLib/skinContactList.h>
#include <iCub/iKin/iKinFwd.h>

#define PRESSURE_THRESHOLD 15.0

using namespace std;

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;

using namespace iCub::skinDynLib;
using namespace iCub::iKin;

class touchDetectorThread: public Thread {

protected:
    
/***************************************************************************/
// CLASS VARIABLES: contact related variables

	int cntctDetect;

	Vector cntctPosWRF;		// Position in world RF
	Vector cntctPosLinkRF;		// Position in i-th link RF 
	Matrix Twl;			// RT matrix to convert the position from i-th link to world RF
	int cntctLinkNum;		// Link number
	SkinPart cntctSkinPart;		// Skin part
	string cntctSkinPartV;		// Skin part verbose

	double cntcPressure;		// Average output of activated taxels
	unsigned int cntcTaxelNum;	// Number of activated taxels
	
	PolyDriver ddR;
    PolyDriver ddL;
	PolyDriver ddT;

	IEncoders *iencsR;
	IEncoders *iencsL;
	IEncoders *iencsT;
	Vector encsR;
	Vector encsL;
	Vector encsT;
	int jntsR;
	int jntsL;
	int jntsT;

	iCubArm *armR;			// Left arm model
	iCubArm *armL;			// Right arm model

/***************************************************************************/
// THREAD PARAMETERS: pointers to the original variables in touchDetector

	BufferedPort<skinContactList> *inPort;  // Must read from /skinManager/skin_events:o
	BufferedPort<Bottle> *outPort;		// Outputs a Bottle with SkinPart (enum) + cntcPosWRF (Vector)
	double *pressureTh;			// Threshold on cntcPressure for a contact to be detected

	string name;
	string robot;

public:

    touchDetectorThread(string &_name, string &_robot, BufferedPort<skinContactList> *in, BufferedPort<Bottle> *out, double *th) { 
	
		name = _name;
		robot = _robot;

		inPort = in;
		outPort = out;
		pressureTh = th;

		cntctDetect = -1;
		
		cntctPosLinkRF = zeros(3); 
	    cntctPosWRF = zeros(3); 
		Twl = eye(4);
		cntctLinkNum = -1;
		cntctSkinPart = SKIN_PART_UNKNOWN; 
		cntctSkinPartV = "";

		cntcPressure = -1;
		cntcTaxelNum = -1;

		armR = new iCubArm("right");
		armL = new iCubArm("left");

	}

    virtual bool threadInit() {
        
	Property OptR;
    OptR.put("robot",  robot.c_str());
    OptR.put("part",   "right_arm");
    OptR.put("device", "remote_controlboard");
    OptR.put("remote",("/"+robot+"/right_arm").c_str());
    OptR.put("local", ("/"+name +"/right_arm").c_str());

    Property OptL;
    OptL.put("robot",  robot.c_str());
    OptL.put("part",   "left_arm");
    OptL.put("device", "remote_controlboard");
    OptL.put("remote",("/"+robot+"/left_arm").c_str());
    OptL.put("local", ("/"+name +"/left_arm").c_str());

	Property OptT;
    OptT.put("robot",  robot.c_str());
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote",("/"+robot+"/torso").c_str());
    OptT.put("local", ("/"+name +"/torso").c_str());

	if (!ddR.open(OptR)) {
        fprintf(stdout,"ERROR: could not open right_arm PolyDriver!\n");
        return false;
    }
    bool ok = 1;
    if (ddR.isValid())
        ok = ok && ddR.view(iencsR);
    if (!ok) {
        fprintf(stdout,"ERROR: Problems acquiring right_arm interfaces!\n");
        return false;
    }

    if (!ddL.open(OptL)) {
        fprintf(stdout,"ERROR: could not open left_arm PolyDriver!\n");
        return false;
    }
    ok = 1;
    if (ddL.isValid())
        ok = ok && ddL.view(iencsL);
    if (!ok) {
        fprintf(stdout,"ERROR: Problems acquiring left_arm interfaces!\n");
        return false;
    }

	if (!ddT.open(OptT)) {
        fprintf(stdout,"ERROR: could not open torso PolyDriver!\n");
        return false;
    }
    ok = 1;
    if (ddT.isValid())
        ok = ok && ddT.view(iencsT);
    if (!ok) {
        fprintf(stdout,"ERROR: Problems acquiring torso interfaces!\n");
        return false;
    }

	iencsR->getAxes(&jntsR);
    encsR.resize(jntsR,0.0);
	iencsL->getAxes(&jntsL);
    encsL.resize(jntsL,0.0);
    iencsT->getAxes(&jntsT);
    encsT.resize(jntsT,0.0);

	armR->releaseLink(0);
	armR->releaseLink(1);
	armR->releaseLink(2);
	
	armL->releaseLink(0);
	armL->releaseLink(1);
	armL->releaseLink(2);

    return true;
    }

    virtual void run() {


		while (isStopping() != true) {
 
			skinContactList *cntctVec = inPort->read(false); // manage the case when cntctVec == NULL i.e. old data
			
			/* Process the skinContactList:
			case 0: NULL --> no new data --> doing nothing
			case 1: empty --> no contact detected --> doing nothing
			case 2: not empty --> at least 1 contact above threshold --> sending: cntctSkinPArt + cntctPosWRF
			case 3: not empty --> 0 contact above threshold --> sending: -1
			*/

			if (cntctVec!=NULL && !cntctVec->empty()) {
			
					for (skinContactList::iterator it = cntctVec->begin(); it!=cntctVec->end(); it++) {
						
						cntctPosLinkRF = it->getCoP();
						cntctLinkNum = it->getLinkNumber();
						cntcPressure = it->getPressure();
						cntcTaxelNum = it->getActiveTaxels();
						cntctSkinPart  = it->getSkinPart();
						cntctSkinPartV = it->getSkinPartName();

						if (cntcPressure>=*pressureTh)
							break;
					}

					if (cntcPressure>=*pressureTh)
						cntctDetect = 0;
					else 
						cntctDetect = -1;

					/* Prepare the output bottle */ 

					Bottle &output = outPort->prepare();
					output.clear();

					if (cntctDetect==0) {

						output.addInt(cntctSkinPart);

						/* Convert the coordinates from link to robot reference frame*/
				
						switch (cntctSkinPart) {
							case SKIN_LEFT_HAND:
							case SKIN_LEFT_FOREARM:
							case SKIN_LEFT_UPPER_ARM: {

								iencsT->getEncoders(encsT.data());
								double tmp = encsT[0];
								encsT[0] = encsT[2];
								encsT[2] = tmp;

								iencsL->getEncoders(encsL.data());

								Vector encsLnoHand = encsL.subVector(0,6);

								Vector qTL(encsT.size()+encsLnoHand.size(),0.0);
								qTL.setSubvector(0,encsT);
								qTL.setSubvector(encsT.size(),encsLnoHand);

								armL -> setAng(qTL*CTRL_DEG2RAD);

								Twl = armL -> getH(cntctLinkNum + encsT.size(), true);

								break;
							}
							case SKIN_RIGHT_HAND:
							case SKIN_RIGHT_FOREARM:
							case SKIN_RIGHT_UPPER_ARM: {

								iencsT->getEncoders(encsT.data());
								double tmp = encsT[0];
								encsT[0] = encsT[2];
								encsT[2] = tmp;
							
								iencsR->getEncoders(encsR.data());

								Vector encsRnoHand = encsR.subVector(0,6);

								Vector qTR(encsT.size()+encsRnoHand.size(),0.0);
								qTR.setSubvector(0,encsT);
								qTR.setSubvector(encsT.size(),encsRnoHand);

								armR -> setAng(qTR*CTRL_DEG2RAD);

								Twl = armR -> getH(cntctLinkNum + encsT.size(), true);

								break;
							} 
							case SKIN_FRONT_TORSO: {
							
								iencsT->getEncoders(encsT.data());
								double tmp = encsT[0];
								encsT[0] = encsT[2];
								encsT[2] = tmp;
							
								armR -> setAng(encsT*CTRL_DEG2RAD);

								Twl = armR -> getH(cntctLinkNum, true);

								break;
							}
							default: {

								fprintf(stdout, "SkinPart: %s\n", cntctSkinPartV.c_str());

								break;
							}
						}

						cntctPosLinkRF.push_back(1);
						cntctPosWRF.push_back(1);
						cntctPosWRF = Twl * cntctPosLinkRF;
						cntctPosLinkRF.pop_back();
						cntctPosWRF.pop_back();

						fprintf(stdout,"CONTACT! skinPart: %s Link: %i Position: %s\n", cntctSkinPartV.c_str(), cntctLinkNum,cntctPosWRF.toString().c_str());
				
						Bottle outputCoord;
						outputCoord.addDouble(cntctPosWRF[0]);
						outputCoord.addDouble(cntctPosWRF[1]);
						outputCoord.addDouble(cntctPosWRF[2]);

						output.addList() = outputCoord;
				
					} else {

					output.addInt(cntctDetect);
					
					}

					outPort->write();
			}

		}

}

    virtual void threadRelease() {    
        
		ddR.close();
		ddL.close();
		ddT.close();

		fprintf(stdout, "DetectTouchThread: closed Device Drivers...\n");

		//delete armR;
		//delete armL;

		//delete iencsL;
		//delete iencsR;
		//delete iencsT;
    }
};

class touchDetector: public RFModule {

protected:

/***************************************************************************/
// MODULE and PORTS

	string robot;
	string name;
	
	string inputPortSubfix, inputPortName;
	string outputPortSubfix, outputPortName;
	string handlerPortName;

	BufferedPort<skinContactList> inputPort;
	BufferedPort<Bottle> outputPort;
	Port handlerPort;

	string helpMessage;

/***************************************************************************/
// CLASS VARIABLES
	
	touchDetectorThread *workThread;

	double pressureThreshold;

public:

	touchDetector() {

		robot = "icubSim";
		name = "touchDetector";
		
		inputPortSubfix	= "/contacts:i";
		outputPortSubfix	= "/contact_pos:o";

		helpMessage = string(getName().c_str());
		helpMessage += "Options:\n";
		helpMessage += "  --context				path:  where to find the called resource (default touchDetector/conf)\n";
		helpMessage += "  --from				from:  name of the .ini file (default touchDetector.ini)\n";
		helpMessage += "  --name				name:  name of the module (default touchDetector)\n";
		helpMessage += "  --robot				robot: name of the robot (default icubSim)\n";
		helpMessage += "  --input_port			input_port:  subfix of the input port (default /contacts:i)\n";
		helpMessage += "  --output_port			output_port:  subfix of the output port (default /contacts_pos:o)\n";
		helpMessage += "  --pressure_thresh		pressure_thresh: threshold on the avg pressure for a contact to be detected (default 15)\n";
		helpMessage += "Commands:\n";
		helpMessage += "stop 					stop the module\n";
		helpMessage += "help 					print options and commands\n";
		helpMessage += "set pressure_thresh <n>	set pressure_thresh to <n> (double)\n";

	}

    bool configure(yarp::os::ResourceFinder &rf) {
        
		/******************** MODULE ****************/
		name = rf.check("name", Value(name), "name (string)").asString();
		setName(name.c_str()); 

		/******************** ROBOT ****************/
		robot = rf.check("robot", Value(robot), "robot (string)").asString();

		/******************** PORTS *****************/
		inputPortName	= "/" + getName(rf.check("input_port", Value(inputPortSubfix), "input_port (string)").asString());
		outputPortName	= "/" + getName(rf.check("output_port", Value(outputPortSubfix), "output_port (string)").asString());
		handlerPortName = "/" + getName() + "/rpc:i";
        
		if (!inputPort.open(inputPortName.c_str())) {
			fprintf(stdout, "%s: unable to open port %s\n", (getName()).c_str(), inputPortName.c_str());
			return false;
		}
		if (!outputPort.open(outputPortName.c_str())) {
			fprintf(stdout, "%s: unable to open port %s\n", (getName()).c_str(), outputPortName.c_str());
			return false;
		}
		if (!handlerPort.open(handlerPortName.c_str())) {
			fprintf(stdout, "%s: Unable to open port %s\n", (getName()).c_str(), handlerPortName.c_str());
			return false;
		}

		attach(handlerPort); // Attach to port
		/*attachTerminal();    // Attach to terminal */

		/************** CLASS VARIABLES *************/

		pressureThreshold = rf.check("pressure_thresh",Value(PRESSURE_THRESHOLD),"pressure_thresh (double)").asDouble();

		workThread = new touchDetectorThread(name, robot, &inputPort, &outputPort, &pressureThreshold);

		bool startOk = workThread->start();
        if (!startOk) {
			delete workThread;
            workThread = NULL;
            fprintf(stdout, "ERROR: detectTouchThread not instantiated\n");
            return false;
        }

		return true ;  
	}

	bool interruptModule() { 

		inputPort.interrupt();
		outputPort.interrupt();
		handlerPort.interrupt();

		return true;
	}

    virtual bool close()
    {
        inputPort.close();
	outputPort.close();
	handlerPort.close();

	fprintf(stdout, "detectTouch: Closed ports..\n");
        if (workThread) {
            workThread->stop();
	    fprintf(stdout, "detectTouch: Stopped thread...\n");
            delete workThread;
            workThread=0;
        }

        return true;
    }

	bool respond(const Bottle& command, Bottle& reply) {

		int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");
		
		reply.clear(); 

		if (command.size()>0) {
			if (command.get(0).asString() == "stop") {
				inputPort.close();
				outputPort.close();
				handlerPort.close();
				fprintf(stdout, "detectTouch: Stopping threads...\n");
				if (workThread) {
					workThread->stop();
					delete workThread;
					workThread=0;
				}
				reply.addVocab(ack);
				return true;
			} else if (command.get(0).asString() == "help") {
				fprintf(stdout, "%s", helpMessage.c_str());
				reply.addVocab(ack);
			} else if ((command.get(0).asString() == "set") && (command.get(1).asString() == "pressure_thresh")) {
				pressureThreshold = command.get(2).asDouble();
				reply.addVocab(ack);
			}
		}

		reply.addVocab(nack);
		return true;
}

    double getPeriod() { return 1.0; }

    bool updateModule() { return true; }
};

int main(int argc, char *argv[]) {

    ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile( "touchDetector.ini" ); //overridden by --from parameter
    rf.setDefaultContext( "touchDetector/conf" );   //overridden by --context parameter
    rf.configure(/*"ICUB_ROOT",*/ argc, argv );

	string optMessage = "Options:\n";
	optMessage =+ " --context			path:  where to find the called resource (default touchDetector/conf)\n";
	optMessage =+ " --from				from:  name of the .ini file (default touchDetector.ini)\n";
	optMessage =+ " --name				name:  name of the module (default touchDetector)\n";
	optMessage += " --robot				robot: name of the robot (default icubSim)\n";
	optMessage =+ " --input_port		input_port:  subfix of the input port (default /contacts:i)\n";
	optMessage =+ " --output_port		output_port:  subfix of the output port (default /contacts_pos:o)\n";
	optMessage =+ " --pressure_thresh   pressure_thresh: threshold on the avg pressure for a contact to be detected (default 15)\n";

	if (rf.check("help")) {    
        fprintf(stdout, "%s", optMessage.c_str());
		return 0;
    }
   
	Network yarp;
    if (!yarp.checkNetwork()) {
        fprintf(stdout,"ERROR: yarp server does not seem available\n");
        return -1;
    }

    touchDetector mod;
    return mod.runModule(rf);
}
