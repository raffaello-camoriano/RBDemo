/* 
 * Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Matej Hoffmann
 * email:  matej.hoffmann@iit.it
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

#include <gsl/gsl_math.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/neuralNetworks.h>

#define DEFAULT_THR_PER     20

#define NOARM               0
#define LEFTARM             1
#define RIGHTARM            2
#define USEDARM             3

#define OPENHAND            0
#define CLOSEHAND           1

#define STATE_IDLE              0
#define STATE_REACH             1
#define STATE_CHECKMOTIONDONE   2
#define STATE_RELEASE           3
#define STATE_WAIT              4

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;


class reachingThread: public RateThread
{
protected:
    ResourceFinder &rf;
    
    string name;
    string robot;
    
    bool useLeftArm;
    bool useRightArm;
    int  armSel;
    
    PolyDriver *drvTorso, *drvLeftArm, *drvRightArm;
    PolyDriver *drvCartLeftArm, *drvCartRightArm;
    
    IEncoders         *encTorso;
    IPositionControl  *posTorso;
    IEncoders         *encArm;
    IPositionControl  *posArm;
    ICartesianControl *cartArm;
    
    BufferedPort<Bottle> inportTargetCoordinates;
    Bottle                 *targetCoordinates;
     
    Vector leftArmReachOffs;
    Vector leftArmHandOrien;
    Vector leftArmJointsStiffness;
    Vector leftArmJointsDamping;

    Vector rightArmReachOffs;
    Vector rightArmHandOrien;
    Vector rightArmJointsStiffness;
    Vector rightArmJointsDamping;
    
    Vector *armReachOffs;
    Vector *armHandOrien;
    
    Vector homePoss, homeVels;
    
    bool wentHome;
    bool leftArmImpVelMode;
    bool rightArmImpVelMode;
    
    bool targetFromRPCset;
    bool targetFromPortSet;

    double trajTime;
    double reachTol;
    double idleTimer, idleTmo;
    double latchTimer;
    double hystThres;
    
    Vector openHandPoss;
    Vector handVels;
    
    Vector torso;
    
    Vector targetPosFromPort;
    Vector targetPosFromRPC;
    Vector targetPos;
    
    Matrix R,Rx,Ry,Rz;

    int state;
    int startup_context_id_left;
    int startup_context_id_right;
       
    void initCartesianCtrl(Vector &sw, Matrix &lim, const int sel=USEDARM)
    {
        ICartesianControl *icart=cartArm;
        Vector dof;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
            {
                drvCartLeftArm->view(icart);
                icart->storeContext(&startup_context_id_left);
                icart->restoreContext(0);
            }
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
            {
                drvCartRightArm->view(icart);
                icart->storeContext(&startup_context_id_right);
                icart->restoreContext(0);
            }
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Initializing %s controller ...\n",type.c_str());

        icart->setTrackingMode(false);
        icart->setTrajTime(trajTime);
        icart->setInTargetTol(reachTol);
        icart->getDOF(dof);

        for (size_t j=0; j<sw.length(); j++)
        {
            dof[j]=sw[j];

            if (sw[j] && (lim(j,0) || lim(j,2)))
            {
                double min, max;
                icart->getLimits(j,&min,&max);

                if (lim(j,0))
                    min=lim(j,1);

                if (lim(j,2))
                    max=lim(j,3);

                icart->setLimits(j,min,max);
                fprintf(stdout,"jnt #%d in [%g, %g] deg\n",(int)j,min,max);
            }
        }

        icart->setDOF(dof,dof);

        fprintf(stdout,"DOF's=( ");
        for (size_t i=0; i<dof.length(); i++)
            fprintf(stdout,"%s ",dof[i]>0.0?"on":"off");
        fprintf(stdout,")\n");
    }
    
    void getSensorData()
    {
         if (encTorso->getEncoders(torso.data())){
            R=rotx(torso[1])*roty(-torso[2])*rotz(-torso[0]);
         }
    }
    
    bool checkPosFromPortInput(Vector &target_pos)
    {
        if (Bottle *targetPosNew=inportTargetCoordinates.read(false))
        {
            if (targetPosNew->size()==3)
            {
                target_pos[0]=targetPosNew->get(0).asDouble();
                target_pos[1]=targetPosNew->get(1).asDouble();
                target_pos[2]=targetPosNew->get(2).asDouble();
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;   
        }
    }
    
    void moveHand(const int action, const int sel=USEDARM)
    {
        IPositionControl *ipos=posArm;
        Vector *poss=NULL;
        string actionStr, type;

        switch (action)
        {
        case OPENHAND:
                poss=&openHandPoss;
                actionStr="Opening";
                break;

        case CLOSEHAND:
                printf("ERROR: CLOSEHAND is not suppported by this module.");
                return;
                break;

        default:
            return;
        }

        if (sel==LEFTARM)
        {    
            drvLeftArm->view(ipos);
            type="left_hand";
        }
        else if (sel==RIGHTARM)
        {    
            drvRightArm->view(ipos);
            type="right_hand";
        }
        else
            type=armSel==LEFTARM?"left_hand":"right_hand";

        fprintf(stdout,"*** %s %s\n",actionStr.c_str(),type.c_str());

        for (size_t j=0; j<handVels.length(); j++)
        {
            int k=homeVels.length()+j;

            ipos->setRefSpeed(k,handVels[j]);
            ipos->positionMove(k,(*poss)[j]);
        }
    }

    
    void openHand(const int sel=USEDARM)
    {
        moveHand(OPENHAND,sel);
    }
    
    void selectArm()
    {
        if (useLeftArm && useRightArm)
        {
            if (state==STATE_REACH)
            {    
                // handle the hysteresis thresholds
                if ((armSel==LEFTARM) && (targetPos[1]>hystThres) ||
                    (armSel==RIGHTARM) && (targetPos[1]<-hystThres))
                {
                    fprintf(stdout,"*** Change arm event triggered\n");
                    state=STATE_CHECKMOTIONDONE;
                    latchTimer=Time::now();
                }
            }
            else if (state==STATE_CHECKMOTIONDONE)
            {
                bool done;
                cartArm->checkMotionDone(&done);
                if (!done)
                {
                    if (Time::now()-latchTimer>3.0*trajTime)
                    {
                        fprintf(stdout,"--- Timeout elapsed => FORCE STOP and CHANGE ARM\n");
                        done=true;
                    }
                }

                if (done)
                {
                    stopControl();
                    steerArmToHome();

                    // swap interfaces
                    if (armSel==RIGHTARM)
                    {
                        armSel=LEFTARM;

                        drvLeftArm->view(encArm);
                        drvLeftArm->view(posArm);
                        drvCartLeftArm->view(cartArm);
                        armReachOffs=&leftArmReachOffs;
                        armHandOrien=&leftArmHandOrien;
                    }
                    else
                    {
                        armSel=RIGHTARM;

                        drvRightArm->view(encArm);
                        drvRightArm->view(posArm);
                        drvCartRightArm->view(cartArm);
                        armReachOffs=&rightArmReachOffs;
                        armHandOrien=&rightArmHandOrien;
                    }

                    fprintf(stdout,"*** Using %s\n",armSel==LEFTARM?"left_arm":"right_arm");
                    stopArmJoints();
                    state=STATE_REACH;
                }
            }
        }
    }
    
    void doReach()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_REACH)
            {
                Vector x=R.transposed()*(targetPos+*armReachOffs);
                limitRange(x);
                x=R*x;
    
                cartArm->goToPose(x,*armHandOrien);
            }
        }
    }
         
    void doIdle()
    {
    }
    
    void steerTorsoToHome()
    {
        Vector homeTorso(3);
        homeTorso.zero();

        Vector velTorso(3);
        velTorso=10.0;

        fprintf(stdout,"*** Homing torso\n");

        posTorso->setRefSpeeds(velTorso.data());
        posTorso->positionMove(homeTorso.data());
    }
    
    void checkTorsoHome(const double timeout=10.0)
    {
        fprintf(stdout,"*** Checking torso home position... ");

        bool done=false;
        double t0=Time::now();
        while (!done && (Time::now()-t0<timeout))
        {
            posTorso->checkMotionDone(&done);
            Time::delay(0.1);
        }

        fprintf(stdout,"*** done\n");
    }

    
    void stopArmJoints(const int sel=USEDARM)
    {
        IEncoders        *ienc=encArm;
        IPositionControl *ipos=posArm;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
            {
                drvLeftArm->view(ienc);
                drvLeftArm->view(ipos);
            }
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
            {
                drvRightArm->view(ienc);
                drvRightArm->view(ipos);
            }
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Stopping %s joints\n",type.c_str());
        for (size_t j=0; j<homeVels.length(); j++)
        {
            double fb;

            ienc->getEncoder(j,&fb);
            ipos->positionMove(j,fb);
        }
    }
     
    void steerArmToHome(const int sel=USEDARM)
    {
        IPositionControl *ipos=posArm;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
                drvLeftArm->view(ipos);
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
                drvRightArm->view(ipos);
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Homing %s\n",type.c_str());
        for (size_t j=0; j<homeVels.length(); j++)
        {
            ipos->setRefSpeed(j,homeVels[j]);
            ipos->positionMove(j,homePoss[j]);
        }

        openHand(sel);
    }
    
     void checkArmHome(const int sel=USEDARM, const double timeout=10.0)
    {
        IPositionControl *ipos=posArm;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
                drvLeftArm->view(ipos);
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
                drvRightArm->view(ipos);
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Checking %s home position... ",type.c_str());

        bool done=false;
        double t0=Time::now();
        while (!done && (Time::now()-t0<timeout))
        {
            ipos->checkMotionDone(&done);
            Time::delay(0.1);
        }

        fprintf(stdout,"*** done\n");
    }
    
    void stopControl()
    {
        if (useLeftArm || useRightArm)
        {
            fprintf(stdout,"stopping control\n");
            cartArm->stopControl();
            Time::delay(0.1);
        }        
    }

    void limitRange(Vector &x)
    {               
        x[0]=x[0]>-0.1 ? -0.1 : x[0];       
    }
    
    Matrix &rotx(const double theta)
    {
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Rx(1,1)=Rx(2,2)=c;
        Rx(1,2)=-s;
        Rx(2,1)=s;

        return Rx;
    }

    Matrix &roty(const double theta)
    {
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Ry(0,0)=Ry(2,2)=c;
        Ry(0,2)=s;
        Ry(2,0)=-s;

        return Ry;
    }

    Matrix &rotz(const double theta)
    {
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Rz(0,0)=Rz(1,1)=c;
        Rz(0,1)=-s;
        Rz(1,0)=s;

        return Rz;
    }
    
    void close()
    {
        delete drvTorso;
        delete drvLeftArm;
        delete drvRightArm;
        delete drvCartLeftArm;
        delete drvCartRightArm;
        
        inportTargetCoordinates.interrupt();
        inportTargetCoordinates.close();
      
    }
    
public:
    reachingThread(const string &_name, ResourceFinder &_rf) : 
                  RateThread(DEFAULT_THR_PER), name(_name), rf(_rf)
    {        
        drvTorso=drvLeftArm=drvRightArm=NULL;
        drvCartLeftArm=drvCartRightArm=NULL;
               
    }
    
    bool threadInit()
    {
        //general part
        robot = "icubSim";
        //getting some default values from contexts/demoGrasp_IIT_ISR/config.ini
        useLeftArm = true;
        useRightArm = true;
        trajTime=2.0;
        reachTol=0.01;
        setRate(DEFAULT_THR_PER); // 20 ms ~ 50 Hz
        hystThres = 0.05;
        idleTmo = 5.0;
                   
        // torso part
        Vector torsoSwitch(3);   torsoSwitch.zero();
        Matrix torsoLimits(3,4); torsoLimits.zero();
        torsoSwitch[0]=1.0; //pitch on 
        torsoSwitch[1]=0.0; //roll off 
        torsoSwitch[2]=1.0; //yaw on 
        torsoLimits(0,2)=1.0; torsoLimits(0,3)=10.0;  //max limit on pitch
              
        // arm parts
        leftArmReachOffs.resize(3,0.0);
        leftArmHandOrien.resize(4,0.0);
        leftArmJointsStiffness.resize(5,0.0);
        leftArmJointsDamping.resize(5,0.0);
        rightArmReachOffs.resize(3,0.0);
        rightArmHandOrien.resize(4,0.0);
        rightArmJointsStiffness.resize(5,0.0);
        rightArmJointsDamping.resize(5,0.0);

         //getting some default values from contexts/demoGrasp_IIT_ISR/config.ini and making them symmetrical    
        leftArmReachOffs[0]=0.03; leftArmReachOffs[1]=-0.08; leftArmReachOffs[2]=-0.03;  
        leftArmHandOrien[0]= -0.03; leftArmHandOrien[1]= 0.7; leftArmHandOrien[2]= -0.7; leftArmHandOrien[3]= 3.0;
        leftArmJointsStiffness[0]=0.5; leftArmJointsStiffness[1]=0.5; leftArmJointsStiffness[2]=0.5; leftArmJointsStiffness[3]=0.2; leftArmJointsStiffness[4]=0.1;
        leftArmJointsDamping[0]=0.06; leftArmJointsDamping[1]=0.06; leftArmJointsDamping[2]=0.06; leftArmJointsDamping[3]=0.02; leftArmJointsDamping[4]=0.0;
        
        rightArmReachOffs[0]=0.03; leftArmReachOffs[1]=0.08; leftArmReachOffs[2]=-0.03;  
        rightArmHandOrien[0]= -0.03; rightArmHandOrien[1]= -0.7; rightArmHandOrien[2]= 0.7; rightArmHandOrien[3]= 3.0;      
        rightArmJointsStiffness[0]=0.5 ; rightArmJointsStiffness[1]=0.5 ; rightArmJointsStiffness[2]=0.5 ; rightArmJointsStiffness[3]=0.2 ; rightArmJointsStiffness[4]=0.1 ;
        rightArmJointsDamping[0]=0.06; rightArmJointsDamping[1]=0.06; rightArmJointsDamping[2]=0.06; rightArmJointsDamping[3]=0.02; rightArmJointsDamping[4]=0.0;
       
        openHandPoss.resize(9,0.0); //the default positions are all zeros
        handVels.resize(9,0.0);
        handVels[0]=20.0; handVels[1]=40.0; handVels[2]=50.0; handVels[3]=50.0; handVels[4]=50.0; handVels[5]=50.0; handVels[6]=50.0; handVels[7]=50.0; handVels[8]=80.0;
        
        homePoss.resize(7,0.0); homeVels.resize(7,0.0);
        homePoss[0]=-30.0; homePoss[1]=30.0; homePoss[2]=0.0; homePoss[3]=45.0; homePoss[4]=0.0; homePoss[5]=0.0; homePoss[6]=0.0;
        homeVels[0]=10.0; homeVels[1]=10.0; homeVels[2]=10.0; homeVels[3]=10.0; homeVels[4]=10.0; homeVels[5]=10.0; homeVels[6]=10.0;
        
        targetPosFromPort.resize(3,0.0);
        targetPosFromRPC.resize(3,0.0);
        targetPos.resize(3,0.0);
       
         // open ports
        inportTargetCoordinates.open((name+"/reachingTarget:i").c_str());
       
        string fwslash="/";

        // open remote_controlboard drivers
        Property optTorso("(device remote_controlboard)");
        Property optLeftArm("(device remote_controlboard)");
        Property optRightArm("(device remote_controlboard)");

        optTorso.put("remote",(fwslash+robot+"/torso").c_str());
        optTorso.put("local",(name+"/torso").c_str());

        optLeftArm.put("remote",(fwslash+robot+"/left_arm").c_str());
        optLeftArm.put("local",(name+"/left_arm").c_str());

        optRightArm.put("remote",(fwslash+robot+"/right_arm").c_str());
        optRightArm.put("local",(name+"/right_arm").c_str());

        drvTorso=new PolyDriver;
        if (!drvTorso->open(optTorso))
        {
            close();
            return false;
        }

        if (useLeftArm)
        {
            drvLeftArm=new PolyDriver;
            if (!drvLeftArm->open(optLeftArm))
            {
                close();
                return false;
            }
        }

        if (useRightArm)
        {
            drvRightArm=new PolyDriver;
            if (!drvRightArm->open(optRightArm))
            {
                close();
                return false;
            }
        }

        // open cartesiancontrollerclient and gazecontrollerclient drivers
        Property optCartLeftArm("(device cartesiancontrollerclient)");
        Property optCartRightArm("(device cartesiancontrollerclient)");
       
        optCartLeftArm.put("remote",(fwslash+robot+"/cartesianController/left_arm").c_str());
        optCartLeftArm.put("local",(name+"/left_arm/cartesian").c_str());
    
        optCartRightArm.put("remote",(fwslash+robot+"/cartesianController/right_arm").c_str());
        optCartRightArm.put("local",(name+"/right_arm/cartesian").c_str());

        if (useLeftArm)
        {
            drvCartLeftArm=new PolyDriver;
            if (!drvCartLeftArm->open(optCartLeftArm))
            {
                close();
                return false;
            }

            if (leftArmImpVelMode)
            {
                IControlMode      *imode;
                IImpedanceControl *iimp;

                drvLeftArm->view(imode);
                drvLeftArm->view(iimp);

                int len=leftArmJointsStiffness.length()<leftArmJointsDamping.length()?
                        leftArmJointsStiffness.length():leftArmJointsDamping.length();

                for (int j=0; j<len; j++)
                {
                    imode->setImpedanceVelocityMode(j);
                    iimp->setImpedance(j,leftArmJointsStiffness[j],leftArmJointsDamping[j]);
                }
            }
        }

        if (useRightArm)
        {
            drvCartRightArm=new PolyDriver;
            if (!drvCartRightArm->open(optCartRightArm))
            {
                close();
                return false;
            }

            if (rightArmImpVelMode)
            {
                IControlMode      *imode;
                IImpedanceControl *iimp;

                drvRightArm->view(imode);
                drvRightArm->view(iimp);

                int len=rightArmJointsStiffness.length()<rightArmJointsDamping.length()?
                        rightArmJointsStiffness.length():rightArmJointsDamping.length();

                for (int j=0; j<len; j++)
                {
                    imode->setImpedanceVelocityMode(j);
                    iimp->setImpedance(j,rightArmJointsStiffness[j],rightArmJointsDamping[j]);
                }
            }
        }

        if (useLeftArm)
        {
            drvLeftArm->view(encArm);
            drvLeftArm->view(posArm);
            drvCartLeftArm->view(cartArm);
            armReachOffs=&leftArmReachOffs;
            armHandOrien=&leftArmHandOrien;
            armSel=LEFTARM;
        }
        else if (useRightArm)
        {
            drvRightArm->view(encArm);
            drvRightArm->view(posArm);
            drvCartRightArm->view(cartArm);
            armReachOffs=&rightArmReachOffs;
            armHandOrien=&rightArmHandOrien;
            armSel=RIGHTARM;
        }
        else
        {
            encArm=NULL;
            posArm=NULL;
            cartArm=NULL;
            armReachOffs=NULL;
            armHandOrien=NULL;
            armSel=NOARM;
        }

        // init
        int torsoAxes;
        encTorso->getAxes(&torsoAxes);
        torso.resize(torsoAxes,0.0);

        targetPos.resize(3,0.0);
        R=Rx=Ry=Rz=eye(3,3);

        initCartesianCtrl(torsoSwitch,torsoLimits,LEFTARM);
        initCartesianCtrl(torsoSwitch,torsoLimits,RIGHTARM);

        // steer the robot to the initial configuration
        stopControl();
        steerTorsoToHome();
        steerArmToHome(LEFTARM);
        steerArmToHome(RIGHTARM);

        idleTimer=Time::now();
        Random::seed((int)idleTimer);

        wentHome=false;
        state=STATE_IDLE;

        return true;
    }

    bool setTargetFromRPC(const Vector target_pos){
        for(int i=0;i<target_pos.size();i++)
        {
            targetPos[i]=target_pos[i];
        }
        targetFromRPCset = true;
        return true;
    }
    
    void run()
    {
        bool newTarget = false;
       
        getSensorData();
        
        targetFromPortSet =  checkPosFromPortInput(targetPosFromPort);
        if(targetFromPortSet || targetFromRPCset){
            newTarget = true;
            if (targetFromRPCset){ //RPC has priority
                targetPos = targetPosFromRPC;
            }
            else{
                targetPos = targetPosFromPort;    
            }
        }
         
        if (newTarget)
        {    
            idleTimer=Time::now();

            if (state==STATE_IDLE)
            {
               fprintf(stdout,"--- Got target => REACHING\n");
                
                wentHome=false;
                state=STATE_REACH;
            }
        }
        else if (((state==STATE_IDLE) || (state==STATE_REACH)) && 
                 ((Time::now()-idleTimer)>idleTmo) && !wentHome)
        {    
            fprintf(stdout,"--- Target timeout => IDLE\n");
            stopControl();
            steerTorsoToHome();
            steerArmToHome(LEFTARM);
            steerArmToHome(RIGHTARM);
            wentHome=true;
            state=STATE_IDLE;
        }
         
        selectArm();
        doReach();
    }

    void threadRelease()
    {
        stopControl();
        steerTorsoToHome();
        steerArmToHome(LEFTARM);
        steerArmToHome(RIGHTARM);

        checkTorsoHome(3.0);
        checkArmHome(LEFTARM,3.0);
        checkArmHome(RIGHTARM,3.0);

        if (useLeftArm)
        {
            ICartesianControl *icart;
            drvCartLeftArm->view(icart);
            icart->restoreContext(startup_context_id_left);

            if (leftArmImpVelMode)
            {
                IControlMode *imode;
                drvLeftArm->view(imode);

                int len=leftArmJointsStiffness.length()<leftArmJointsDamping.length()?
                        leftArmJointsStiffness.length():leftArmJointsDamping.length();

                for (int j=0; j<len; j++)
                    imode->setVelocityMode(j);
            }
        }

        if (useRightArm)
        {
            ICartesianControl *icart;
            drvCartRightArm->view(icart);
            icart->restoreContext(startup_context_id_right);

            if (rightArmImpVelMode)
            {
                IControlMode *imode;
                drvRightArm->view(imode);

                int len=rightArmJointsStiffness.length()<rightArmJointsDamping.length()?
                        rightArmJointsStiffness.length():rightArmJointsDamping.length();

                for (int j=0; j<len; j++)
                    imode->setVelocityMode(j);
            }
        }

        close();
    }
    
};


class reachingModule: public RFModule
{
protected:
    reachingThread *thr;    
    Port           rpcPort;

public:
    reachingModule() { }

    bool configure(ResourceFinder &rf)
    {
        thr=new reachingThread(getName().c_str(),rf);
        if (!thr->start())
        {
            delete thr;    
            return false;
        }

        rpcPort.open(getName("/rpc"));
        attach(rpcPort);

        return true;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        Vector target_pos;
        int ack =Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        target_pos.resize(3,0.0);
        
        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case VOCAB2('g','o'):
                {
                    
                    int res=Vocab::encode("new target");
                    if (command.size()==4){ //"go x y z"
                        target_pos(0)=command.get(1).asDouble();
                        target_pos(1)=command.get(2).asDouble();
                        target_pos(2)=command.get(3).asDouble();
                        if (thr -> setTargetFromRPC(target_pos))
                        {
                            reply.addVocab(ack);
                            reply.addVocab(VOCAB2('g','o'));
                            reply.addDouble(target_pos(0));
                            reply.addDouble(target_pos(1));
                            reply.addDouble(target_pos(2));
                        }
                        else{
                            reply.addVocab(nack);
                        }
                    }
                    else{
                        reply.addVocab(nack);
                    }
                    
                    reply.addVocab(res);
                    return true;
                }
               
                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    
    bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        thr->stop();
        delete thr;

        return true;
    }

    double getPeriod()    { return 0.05;  } // 0.05 s = 50 ms ~ 20 Hz
    bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)
    
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("reachModule");
    rf.setDefaultConfigFile("reachConfig.ini");
    rf.configure(argc,argv);

    reachingModule mod;
    
    cout << "Configuring and starting reaching module. \n";
    return mod.runModule(rf);
}
