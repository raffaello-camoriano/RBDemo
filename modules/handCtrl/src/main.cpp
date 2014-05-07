/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Raffaello Camoriano
 * email: raffaello.camoriano@iit.it
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
\defgroup handCtrl
 
Example of grasping module based upon \ref ActionPrimitives 
library. 

Copyright (C) 2010 RobotCub Consortium
 
Author: Raffaello Camoriano

CopyPolicy: Released under the terms of the GNU GPL v2.0. 

\section intro_sec Description 
A module that makes use of \ref ActionPrimitives 
library to open and close either of the hands.

Scenario:

1) Both hands are opened at startup

2) A bottle containing the code of the hand to be closed
is received
 
3) The corresponding hand is closed by calling the close_hand sequence

4) The hand is opened again by the user via rpc:i
 
\section lib_sec Libraries 
- YARP libraries. 
- \ref ActionPrimitives library.  
- \ref SkinDyn library.  

#\section parameters_sec Parameters
 
\section portsa_sec Ports Accessed
The robot interface is assumed to be operative.
 
\section portsc_sec Ports Created 
Aside from the internal ports created by \ref ActionPrimitives 
library, we also have: 
 
- \e /handCtrl/handToBeClosed:i receives a bottle containing the code associated to the hand to close

 
- \e /handCtrl/rpc:i remote procedure call.

Recognized remote commands:
    - 'open_left_hand'
    - 'open_right_hand'
    - 'close_left_hand'
    - 'close_right_hand'

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files 
--grasp_model_type \e type 
- specify the grasp model type according to the \ref 
  ActionPrimitives documentation.
 
--grasp_model_left_file \e file 
- specify the path to the file containing the grasp model 
  options for the left hand.

--grasp_model_right_file \e file 
- specify the path to the file containing the grasp model 
  options for the right hand.
 
--handCtrl_hand_sequences_file \e file 
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
robot                           icub
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
\endcode 

\section tested_os_sec Tested OS
Windows

\author Raffaello Camoriano
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
#include <yarp/os/Vocab.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/conf/system.h>
#include <iCub/perception/models.h>
#include <iCub/action/actionPrimitives.h>
#include <iCub/skinDynLib/common.h>            // Common skin parts codes

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
class handCtrl: public RFModule
{
protected:
    AFFACTIONPRIMITIVESLAYER *actionL;    // Action list associated to the left hand
    AFFACTIONPRIMITIVESLAYER *actionR;    // Action list associated to the right hand
    BufferedPort<Bottle>      inPort;
    Port                      rpcPort;

    bool firstRun;

public:
    /************************************************************************/
    handCtrl()
    {
        actionL=NULL;
        actionR=NULL;
        firstRun=true;
    }

    bool respond(const Bottle &      command,
                 Bottle &      reply)
    {
        // This method is called when a command string is sent via RPC

        // Get command string
        string receivedCmd = command.get(0).asString().c_str();
        
        // Stop current motion and clear actions queue
        actionL->stopControl();
        actionR->stopControl();
        actionL->clearActionsQueue();
        actionR->clearActionsQueue();

        bool f;
        int responseCode;   //Will contain Vocab-encoded response

        reply.clear();  // Clear reply bottle

        if (receivedCmd == "open_left_hand")
        {

            actionL->pushAction("open_hand");
            actionL->checkActionsDone(f,true);
            actionL->areFingersInPosition(f);    // Check for obstructing (grasped) objects
            
            if (!f)
            {
                //Encode response
                responseCode = Vocab::encode("nack");
                reply.addVocab(responseCode);
                //reply.addVocab(VOCAB4('n','a','c','k'));
            }
            else
            {
                //Encode response
                responseCode = Vocab::encode("ack");
                reply.addVocab(responseCode);
                //reply.addVocab(VOCAB3('a','c','k'));
            }
        }

        else if (receivedCmd == "open_right_hand")
        {
            actionR->pushAction("open_hand");
            actionR->checkActionsDone(f,true);
            actionR->areFingersInPosition(f);    // Check for obstructing (grasped) objects
            
            if (!f)
            {
                 //Encode response
                responseCode = Vocab::encode("nack");
                reply.addVocab(responseCode);
            }
            else
            {
                 //Encode response
                responseCode = Vocab::encode("ack");
                reply.addVocab(responseCode);
            }
        }

        else if (receivedCmd == "close_right_hand")
        {
            actionR->pushAction("close_hand");
            actionR->checkActionsDone(f,true);
            actionR->areFingersInPosition(f);    // Check for obstructing (grasped) objects
            
            if (!f)
            {
                 //Encode response
                responseCode = Vocab::encode("nack");
                reply.addVocab(responseCode);
            }
            else
            {
                 //Encode response
                responseCode = Vocab::encode("ack");
                reply.addVocab(responseCode);
            }
        }

        else if (receivedCmd == "close_left_hand")
        {
            actionL->pushAction("close_hand");
            actionL->checkActionsDone(f,true);
            actionL->areFingersInPosition(f);    // Check for obstructing (grasped) objects
            
            if (!f)
            {
                 //Encode response
                responseCode = Vocab::encode("nack");
                reply.addVocab(responseCode);
            }
            else
            {
                 //Encode response
                responseCode = Vocab::encode("ack");
                reply.addVocab(responseCode);
            }
        }

        return true;
    }

    bool configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();
        setName(name.c_str());

        Property config;
        config.fromConfigFile(rf.findFile("from").c_str());
        Bottle &bGeneral=config.findGroup("general");
        if (bGeneral.isNull())
        {
            cout<<"Error: group general is missing!"<<endl;

            return false;
        }

        // Parsing general config options for both hands

        Property optionL(bGeneral.toString().c_str());    // Left
        optionL.put("local",name.c_str());    //module name
        optionL.put("robot",rf.findGroup("general").find("robot").asString().c_str());
        optionL.put("part","left_arm");
        optionL.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
        optionL.put("grasp_model_file",rf.findFile("grasp_model_left_file").c_str());
        optionL.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());  

        cout << "-- Option Left: " << optionL.toString() << endl;
        
        Property optionR(bGeneral.toString().c_str());    // Right
        optionR.put("local",name.c_str());    //module name
        optionR.put("robot",rf.findGroup("general").find("robot").asString().c_str());
        optionR.put("part","right_arm");
        optionR.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
        optionR.put("grasp_model_file",rf.findFile("grasp_model_right_file").c_str());
        optionR.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());    

        cout << "-- Option Right: " << optionR.toString() << endl;

        cout<<"***** Instantiating primitives for left hand"<<endl;
        actionL = new AFFACTIONPRIMITIVESLAYER(optionL);
        cout<<"***** Instantiating primitives for right hand"<<endl;
        actionR=new AFFACTIONPRIMITIVESLAYER(optionR);
        if (!actionR->isValid() || !actionL->isValid() )
        {
            delete actionR;
            actionR = NULL;
            delete actionL;
            actionL = NULL;
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
        inPort.open((fwslash+name+"/handToBeClosed:i").c_str());
        cout << "inPort opened" << endl;
        rpcPort.open((fwslash+name+"/rpc:i").c_str());
        cout << "rpcPort opened" << endl;

        // Attach rpcPort to the respond() method
        attach(rpcPort);
        
        // Check whether the grasp model is calibrated,
        // otherwise calibrate it and save the results
        
        // Left hand
        Model *modelL;
        actionL->getGraspModel(modelL);
        cout << "DEBUG ################### Got graspModel" << endl;
        if (modelL!=NULL)
        {
            cout << "DEBUG ################### modelL != NULL" << endl;
            cout << "modelL->isCalibrated() = " << modelL->isCalibrated() << endl;
            if (!modelL->isCalibrated())
            {
                cout << "DEBUG ################### modelL is not calibrated" << endl;
                Property prop("(finger all_parallel)");
                modelL->calibrate(prop);
                cout << "DEBUG ################### Tried to calibrate..." << endl;
                string fileName=rf.getHomeContextPath();
                fileName+="/";
                fileName+=optionL.find("grasp_model_left_file").asString().c_str();

                ofstream fout;
                fout.open(fileName.c_str());
                modelL->toStream(fout);
                fout.close();
            }
        }

        // Right hand
        Model *modelR;
        actionR->getGraspModel(modelR);
        if (modelR!=NULL)
        {
            if (!modelR->isCalibrated())
            {
                Property prop("(finger all_parallel)");
                modelR->calibrate(prop);

                string fileName=rf.getHomeContextPath();
                fileName+="/";
                fileName+=optionR.find("grasp_model_right_file").asString().c_str();

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
        {
            delete actionL;
            actionL = NULL;
        }
         
        if (actionR!=NULL)
        {
            delete actionR;
            actionR = NULL;
        }
        
        // Close ports
        inPort.close();
        cout << "inPort closed" << endl;
        rpcPort.close();
        cout << "rpcPort closed" << endl;

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

            /*        NOTE: We are using the following enums from iCub/skinDynLib/common.h
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
                actionL->areFingersInPosition(fl);    // Check for obstructing (grasped) objects
                actionL->pushWaitState(5);      // Wait 5 seconds
                actionL->pushAction("open_hand");
                actionL->checkActionsDone(fl,true);
                actionL->areFingersInPosition(fl);    // Check for obstructing (grasped) objects
            }
            else if ( handSide == iCub::skinDynLib::SKIN_RIGHT_HAND)
            {
                // Right hand
                actionR->pushAction("close_hand");
                actionR->checkActionsDone(fr,true);
                actionR->areFingersInPosition(fr);    // Check for obstructing (grasped) objects
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

        // Interrupt any blocking reads on the input port
        inPort.interrupt();
        cout << "inPort interrupted" << endl;

        // Interrupt any blocking reads on the input port        
        rpcPort.interrupt();
        cout << "rpcPort interrupted" << endl;

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
    rf.setDefaultConfigFile("handCtrl_config.ini");
    rf.setDefaultContext("handCtrl");
    rf.setDefault("grasp_model_type","tactile");    // Check this parameter, does it correspond to the one stored in grasp_model_* -> name?
                                                    // If so, one default option for each hand may be needed.
    rf.setDefault("grasp_model_left_file","grasp_model_left.ini");
    rf.setDefault("grasp_model_right_file","grasp_model_right.ini");
    rf.setDefault("hand_sequences_file","handCtrl_hand_sequences.ini");
    rf.setDefault("name","handCtrl");
    rf.configure(argc,argv);

    handCtrl mod;
    return mod.runModule(rf);
}