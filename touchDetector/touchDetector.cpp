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

#include <yarp/math/Math.h>

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

	Vector cntctPosWRF;			// Position in world RF
	Vector cntctPosLinkRF;		// Position in i-th link RF 
	Matrix Twl;					// RT matrix to convert the position from i-th link to world RF
	int cntctLinkNum;			// Link number
	SkinPart cntctSkinPart;		// Skin part
	string cntctSkinPartV;		// Skin part verbose

	double cntcPressure;		// Average output of activated taxels
	unsigned int cntcTaxelNum;	// Number of activated taxels
	
	iCubArm *armR;				// Left arm model
	iCubArm *armL;				// Right arm model

	/***************************************************************************/
    // THREAD PARAMETERS: pointers to the original variables in touchDetector

	BufferedPort<skinContactList> *inPort;  // Must read from /skinManager/skin_events:o
	BufferedPort<Bottle> *outPort;			// Outputs a Bottle with SkinPart (enum) + cntcPosWRF (Vector)
	double *pressureTh;						// Threshold on cntcPressure for a contact to be detected

public:

    touchDetectorThread::touchDetectorThread(BufferedPort<skinContactList> *in, BufferedPort<Bottle> *out, double *th) { 
	
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
        
        return true;
    }

    virtual void run() {


		while (isStopping() != true) {
 
			skinContactList *cntctVec = inPort->read(false); // manage the case when cntctVec == NULL i.e. old data
			
			/* Process the skinContactList:
			case 0: NULL --> no new data --> doing nothing (?)
			case 1: empty --> no contact detected --> sending: -1
			case 2: not empty --> at least 1 contact above threshold --> sending: cntctSkinPArt + cntctPosWRF
			case 3: not empty --> 0 contact above threshold --> sending: -2
			*/

			if (cntctVec!=NULL) {
			
				if (cntctVec->empty()) {

					cntctDetect = -1;

				} else {
				
					skinContactList::iterator it = cntctVec->begin();

					cntctPosLinkRF = it->getCoP();
					cntctLinkNum = it->getLinkNumber();
					cntctSkinPart  = it->getSkinPart();
					cntctSkinPartV = it->getSkinPartName();
					cntcPressure = it->getPressure();
					cntcTaxelNum = it->getActiveTaxels();

					while (cntcPressure<PRESSURE_THRESHOLD && it!=cntctVec->end()) {
						it++;

						cntctPosLinkRF = it->getCoP();
						cntctLinkNum = it->getLinkNumber();
						cntcPressure = it->getPressure();
						cntcTaxelNum = it->getActiveTaxels();
						cntctSkinPart  = it->getSkinPart();
						cntctSkinPartV = it->getSkinPartName();
					}

					if (cntcPressure>=*pressureTh)
						cntctDetect = 0;
					else 
						cntctDetect = -2;

				}

				/* Prepare the output bottle */ 

				Bottle &output = outPort->prepare();
				output.clear();

				if (!cntctDetect) {

					output.addInt(cntctSkinPart);

					/* Convert the coordinates from link to robot reference frame*/
				
					switch (cntctSkinPart) {
						case SKIN_LEFT_HAND:
						case SKIN_LEFT_FOREARM:
						case SKIN_LEFT_UPPER_ARM:
						case SKIN_FRONT_TORSO:
							Twl = armL -> getH(cntctLinkNum, true);
						case SKIN_RIGHT_HAND:
						case SKIN_RIGHT_FOREARM:
						case SKIN_RIGHT_UPPER_ARM:
							Twl = armR -> getH(cntctLinkNum, true);
						default:
							fprintf(stdout, "SkinPart not in arms neither torso: %s\n", cntctSkinPartV);
					}

					cntctPosLinkRF.push_back(1);
					cntctPosWRF.push_back(1);
					cntctPosWRF = Twl * cntctPosLinkRF;
					cntctPosLinkRF.pop_back();
					cntctPosWRF.pop_back();

					fprintf(stdout,"CONTACT! skinPart: %s Link: %i Position: %s\n", cntctSkinPartV, cntctLinkNum,cntctPosLinkRF.toString().c_str());
				
					Bottle outputCoord;
					outputCoord.addDouble(cntctPosWRF[0]);
					outputCoord.addDouble(cntctPosWRF[1]);
					outputCoord.addDouble(cntctPosWRF[2]);

					output.addList() = outputCoord;
				
				} else {

				output.addInt(cntctDetect);
				
				}

			}

			outPort->write();
		
		}

	}

    virtual void threadRelease() {    
        
		delete armR;
		delete armL;
    }
};

class touchDetector: public RFModule {

protected:

    /***************************************************************************/
	// MODULE and PORTS

	string module;
	
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

	touchDetector::touchDetector() {

		module = "touchDetector";
		
		inputPortSubfix	= "/contacts:i";
		outputPortSubfix	= "/contact_pos:o";

		helpMessage = string(getName().c_str());
		helpMessage += "Options:\n";
		helpMessage += "  --context				path:  where to find the called resource (default touchDetector/conf)\n";
		helpMessage += "  --from				from:  name of the .ini file (default touchDetector.ini)\n";
		helpMessage += "  --name				name:  name of the module (default touchDetector)\n";
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
		module = rf.check("name", Value(module), "name (string)").asString();
		setName(module.c_str()); 

		/******************** PORTS *****************/
		inputPortName	= "/" + getName(rf.check("input_port", Value(inputPortSubfix), "input_port (string)").asString());
		outputPortName	= "/" + getName(rf.check("output_port", Value(outputPortSubfix), "output_port (string)").asString());
		handlerPortName = "/" + getName();
        
		if (!inputPort.open(inputPortName.c_str())) {
			fprintf(stdout, "%s: unable to open port %s\n", getName(), inputPortName);
			return false;
		}
		if (!outputPort.open(outputPortName.c_str())) {
			fprintf(stdout, "%s: unable to open port %s\n", getName(), outputPortName);
			return false;
		}
		if (!handlerPort.open(handlerPortName.c_str())) {
			fprintf(stdout, "%s: Unable to open port %s\n", getName(), handlerPortName);
			return false;
		}

		attach(handlerPort); // Attach to port
		attachTerminal();    // Attach to terminal

		/************** CLASS VARIABLES *************/

		pressureThreshold = rf.check("pressure_thresh",Value(PRESSURE_THRESHOLD),"pressure_thresh (double)").asDouble();

		workThread = new touchDetectorThread(&inputPort, &outputPort, &pressureThreshold);

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

		fprintf(stdout, "detectTouch: Stopping threads...\n");
        if (workThread) {
            workThread->stop();
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
				fprintf(stdout, "%s", helpMessage);
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
    rf.configure("ICUB_ROOT", argc, argv );

	string optMessage = "Options:\n";
	optMessage =+ " --context			path:  where to find the called resource (default touchDetector/conf)\n";
	optMessage =+ " --from				from:  name of the .ini file (default touchDetector.ini)\n";
	optMessage =+ " --name				name:  name of the module (default touchDetector)\n";
	optMessage =+ " --input_port		input_port:  subfix of the input port (default /contacts:i)\n";
	optMessage =+ " --output_port		output_port:  subfix of the output port (default /contacts_pos:o)\n";
	optMessage =+ " --pressure_thresh   pressure_thresh: threshold on the avg pressure for a contact to be detected (default 15)\n";

	if (rf.check("help")) {    
        fprintf(stdout, "%s", optMessage);
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