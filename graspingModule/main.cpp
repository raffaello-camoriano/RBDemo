/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/** 
\defgroup ationPrimitivesExample actionPrimitivesExample
 
@ingroup icub_module  
 
Example of grasping module based upon \ref ActionPrimitives 
library. 

Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0. 

\section intro_sec Description 
An example module that makes use of \ref ActionPrimitives 
library in order to execute a sequence of simple actions: 
reaches for an object, tries to grasp it, then lifts it and 
finally releases it. 
 
1) A bottle containing the 3-d position of the object to grasp 
is received (let be x1).
 
2) To this 3-d point a systematic offset is added in order to
compensate for the uncalibrated kinematics of the arm (let be
x2=x1+systematic_offset). 
 
3) The robot reaches for a position located on top of the object 
with the specified orientation (let be reach(x2+grasp_disp,o)). 
 
4) The robot grasps the object (let be reach(x2,o)). 
 
5) The hand is closed. 
 
6) The robot lifts the object to a specified location (let be 
reach(x2+lift_displacement,o)). 
 
7) The robot releases the grasped object. 
 
8) The robot steers the arm to home position. 
 
\note A video on iCub grasping objects can be seen <a 
    href="http://wiki.icub.org/misc/icubvideos/icub_grasps_sponges.wmv">here</a>.
  
\section lib_sec Libraries 
- YARP libraries. 
- \ref ActionPrimitives library.  

\section parameters_sec Parameters
--name \e name
- specify the module name, which is \e ActionPrimitivesMod by
  default.
 
\section portsa_sec Ports Accessed
The robot interface is assumed to be operative.
 
\section portsc_sec Ports Created 
Aside from the internal ports created by \ref ActionPrimitives 
library, we also have: 
 
- \e /<modName>/in receives a bottle containing the 3-d position 
  of the object to grasp.
 
- \e /<modName>/rpc remote procedure call. 
    Recognized remote commands:
    -'quit' quit the module
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files 
--grasp_model_type \e type 
- specify the grasp model type according to the \ref 
  ActionPrimitives documentation.
 
--grasp_model_file \e file 
- specify the path to the file containing the grasp model 
  options.
 
--hand_sequences_file \e file 
- specify the path to the file containing the hand motion 
  sequences relative to the current context ( \ref
  ActionPrimitives ).
 
--from \e file 
- specify the configuration file (use \e --context option to 
  select the current context).
 
The configuration file passed through the option \e --from
should look like as follows:
 
\code 
[general]
// options used to open a ActionPrimitives object 
robot                           icubSim
thread_period                   50
default_exec_time               3.0
reach_tol                       0.007
verbosity                       on 
torso_pitch                     on
torso_roll                      off
torso_yaw                       on
torso_pitch_max                 30.0 
tracking_mode                   off 
verbosity                       on 
 
[arm_dependent]
grasp_orientation               (-0.171542 0.124396 -0.977292 3.058211)
grasp_displacement              (0.0 0.0 0.05)
systematic_error_displacement   (-0.03 -0.07 -0.02)
lifting_displacement            (0.0 0.0 0.2)
home_position                   (-0.29 -0.21 0.11)
home_orientation                (-0.029976 0.763076 -0.645613 2.884471) 
\endcode 

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <deque>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <iCub/perception/models.h>
#include <iCub/action/actionPrimitives.h>
#include <iCub/skinDynLib/common.h>			// Common skin parts codes

//#define USE_LEFT    0
//#define USE_RIGHT   1

#define AFFACTIONPRIMITIVESLAYER    ActionPrimitivesLayer1

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::perception;
using namespace iCub::action;
//using namespace iCub::skinDynLib;


/************************************************************************/
class ExampleModule: public RFModule
{
protected:
    AFFACTIONPRIMITIVESLAYER *actionL;	// Action list associated to the left hand
    AFFACTIONPRIMITIVESLAYER *actionR;	// Action list associated to the right hand
    BufferedPort<Bottle>      inPort;
    Port                      rpcPort;

    // Vector graspOrien;
    // Vector graspDisp;
    // Vector dOffs;
    // Vector dLift;
    // Vector home_x;

    bool firstRun;

public:
    /************************************************************************/
    ExampleModule()
    {
        // graspOrien.resize(4);
        //graspDisp.resize(3);
        //dOffs.resize(3);
        //dLift.resize(3);
        //home_x.resize(3);

        // default values for arm-dependent quantities
        // graspOrien[0]=-0.171542;
        // graspOrien[1]= 0.124396;
        // graspOrien[2]=-0.977292;
        // graspOrien[3]= 3.058211;

        // graspDisp[0]=0.0;
        // graspDisp[1]=0.0;
        // graspDisp[2]=0.05;

        // dOffs[0]=-0.03;
        // dOffs[1]=-0.07;
        // dOffs[2]=-0.02;

        // dLift[0]=0.0;
        // dLift[1]=0.0;
        // dLift[2]=0.15;
        
        // home_x[0]=-0.29;
        // home_x[1]=-0.21;
        // home_x[2]= 0.11;

        actionL=NULL;
        actionR=NULL;
        firstRun=true;
    }

    /************************************************************************/
    void getArmDependentOptions(Bottle &b, Vector &_gOrien, Vector &_gDisp,
                                Vector &_dOffs, Vector &_dLift, Vector &_home_x)
    {
        if (Bottle *pB=b.find("grasp_orientation").asList())
        {
            int sz=pB->size();
            int len=_gOrien.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gOrien[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("grasp_displacement").asList())
        {
            int sz=pB->size();
            int len=_gDisp.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gDisp[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("systematic_error_displacement").asList())
        {
            int sz=pB->size();
            int len=_dOffs.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dOffs[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("lifting_displacement").asList())
        {
            int sz=pB->size();
            int len=_dLift.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dLift[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("home_position").asList())
        {
            int sz=pB->size();
            int len=_home_x.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _home_x[i]=pB->get(i).asDouble();
        }
    }

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();
        setName(name.c_str());

        Property config; config.fromConfigFile(rf.findFile("from").c_str());
        Bottle &bGeneral=config.findGroup("general");
        if (bGeneral.isNull())
        {
            cout<<"Error: group general is missing!"<<endl;
            return false;
        }

        // parsing general config options
        Property optionL(bGeneral.toString().c_str());	// Left
		optionL.put("local",name.c_str());	//left tag?
        optionL.put("part","left_arm");
        optionL.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
        optionL.put("grasp_model_file",rf.findFile("grasp_model_left_file").c_str());
        optionL.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());    

		cout << "-------------" << optionL.toString() << endl;
		
        Property optionR(bGeneral.toString().c_str());	// Right
        optionR.put("local",name.c_str());	//right tag?
        optionR.put("part","right_arm");
        optionR.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
        optionR.put("grasp_model_file",rf.findFile("grasp_model_right_file").c_str());
        optionR.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());    
    

        // parsing arm dependent config options
        // Bottle &bArm=config.findGroup("arm_dependent");
        // getArmDependentOptions(bArm,graspOrien,graspDisp,dOffs,dLift,home_x);

        cout<<"***** Instantiating primitives for left hand"<<endl;
        actionL = new AFFACTIONPRIMITIVESLAYER(optionL);
        if (!actionL->isValid())
        {
            delete actionL;
            return false;
        }

        cout<<"***** Instantiating primitives for right hand"<<endl;
        actionR=new AFFACTIONPRIMITIVESLAYER(optionR);
        if (!actionR->isValid())
        {
            delete actionR;
            return false;
        }
		
		// Get available hand sequence keys for left hand and print them
        deque<string> qL = actionL->getHandSeqList();
        cout<<"***** List of available left hand sequence keys:"<<endl;
        for (size_t i=0; i<qL.size(); i++)
            cout<<qL[i]<<endl;
			
		// Get available hand sequence keys for right hand and print them
        deque<string> qR = actionR->getHandSeqList();
        cout<<"***** List of available right hand sequence keys:"<<endl;
        for (size_t i=0; i<qR.size(); i++)
            cout<<qR[i]<<endl;
			
		// Open ports
        string fwslash="/";
        inPort.open((fwslash+name+"/in").c_str());
        rpcPort.open((fwslash+name+"/rpc").c_str());
        attach(rpcPort);

        // check whether the grasp model is calibrated,
        // otherwise calibrate it and save the results
		
		// Left hand
        Model *modelL; actionL->getGraspModel(modelL);
        if (modelL!=NULL)
        {
            if (!modelL->isCalibrated())
            {
                Property prop("(finger all_parallel)");
                modelL->calibrate(prop);

                string fileName=rf.getHomeContextPath();
                fileName+="/";
                fileName+=optionL.find("grasp_model_file_left").asString().c_str();

                ofstream fout;
                fout.open(fileName.c_str());
                modelL->toStream(fout);
                fout.close();
            }
        }

		// Right hand
        Model *modelR; actionR->getGraspModel(modelR);
        if (modelR!=NULL)
        {
            if (!modelR->isCalibrated())
            {
                Property prop("(finger all_parallel)");
                modelR->calibrate(prop);

                string fileName=rf.getHomeContextPath();
                fileName+="/";
                fileName+=optionR.find("grasp_model_file_right").asString().c_str();

                ofstream fout;
                fout.open(fileName.c_str());
                modelR->toStream(fout);
                fout.close();
            }
        }

        return true;
    }

    /************************************************************************/
    bool close()
    {
        if (actionL!=NULL)
            delete actionL;
			
        if (actionR!=NULL)
            delete actionR;

		// Close ports
        inPort.close();
        rpcPort.close();

        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /************************************************************************/
    void init()
    {
        bool f;

		// Open the hands at startup
		
		// Left hand
        actionL->pushAction("open_hand");
        actionL->checkActionsDone(f,true);
		
		
		// Right hand
        actionR->pushAction("open_hand");
        actionR->checkActionsDone(f,true);
		
        //actionL->enableArmWaving(home_x);		// No required arm waving for hand closure (?)
    }

    // we don't need a thread since the actions library already
    // incapsulates one inside dealing with all the tight time constraints
    /************************************************************************/
    bool updateModule()
    {
        // do it only once
        if (firstRun)
        {
            init();
            firstRun=false;
        }

        // Wait for closure command
        Bottle *b = inPort.read();    // blocking call

        if (b!=NULL)
        {
            iCub::skinDynLib::SkinPart handSide;
            bool fr;
			bool fl;
			
			// Get the code of the hand to be closed
            handSide = static_cast<iCub::skinDynLib::SkinPart>(b->get(0).asInt());

/*		NOTE: We are using the following enums from iCub/skinDynLib/common.h

		enum SkinPart { 
			SKIN_PART_UNKNOWN=0, 
			SKIN_LEFT_HAND, SKIN_LEFT_FOREARM, SKIN_LEFT_UPPER_ARM, 
			SKIN_RIGHT_HAND, SKIN_RIGHT_FOREARM, SKIN_RIGHT_UPPER_ARM, 
			SKIN_FRONT_TORSO, 
			SKIN_PART_ALL, SKIN_PART_SIZE
		};
*/

            // grasp with either of the hands (wait until it's done)

			if ( handSide == iCub::skinDynLib::SKIN_LEFT_HAND)
			{
				// Left hand
				actionL->pushAction("close_hand");
				actionL->checkActionsDone(fl,true);
				actionL->areFingersInPosition(fl);	// Check for obstructing (grasped) objects		
			}
			else if ( handSide == iCub::skinDynLib::SKIN_RIGHT_HAND)
			{
				// Right hand
				actionR->pushAction("close_hand");
				actionR->checkActionsDone(fr,true);
				actionR->areFingersInPosition(fr);	// Check for obstructing (grasped) objects
			}

            // if fingers are not in position,
            // it's likely that we've just grasped
            // something, so print it!
			
			// Left
            if (!fl)
                cout<<"Wow, got something with left hand!"<<endl;
            else
                cout<<"Sorry :( ... nothing to grasp with left hand"<<endl;
			
			// Right
            if (!fr)
                cout<<"Wow, got something with right hand!"<<endl;
            else
                cout<<"Sorry :( ... nothing to grasp with right hand"<<endl;				
        }

        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        // since a call to checkActionsDone() blocks
        // the execution until it's done, we need to 
        // take control and exit from the waiting state
        actionL->syncCheckInterrupt(true);        
		actionR->syncCheckInterrupt(true);        

        inPort.interrupt();
        rpcPort.interrupt();

        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP server not available!"<<endl;
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("graspingModule");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("grasp_model_type","tactile");
    rf.setDefault("grasp_model_left_file","grasp_model_left.ini");
    rf.setDefault("grasp_model_right_file","grasp_model_right.ini");
    rf.setDefault("hand_sequences_file","hand_sequences.ini");
    rf.setDefault("name","graspingModule");
    rf.configure(argc,argv);

    ExampleModule mod;
    return mod.runModule(rf);
}