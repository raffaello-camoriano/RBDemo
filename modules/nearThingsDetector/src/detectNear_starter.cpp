#include <stdio.h>
#include <yarp/os/Network.h>
#include "detectNear.h"
#include <yarp/os/RFModule.h>

using namespace yarp::os;

class detectNearModule: public RFModule
{
   DetectNear detectNear;
public:

    bool configure(ResourceFinder &rf)
    {
        return detectNear.open(rf);
	}

    double getPeriod()
    {
        return 0;
    }
    
    bool updateModule()
    { 
        detectNear.loop();
        return true; 
    }

    bool interruptModule()
    {
        fprintf(stderr, "Interrupting\n");
        detectNear.interrupt();
        return true;
    }

    bool close()
    {
        fprintf(stderr, "Calling close\n");
        detectNear.close();
        return true;
    }

    //void respond();

};

int main(int argc, char *argv[]) 
{
    Network yarp;

    detectNearModule module;
    ResourceFinder rf;
	rf.setVerbose();
    rf.configure("ICUB_ROOT", argc, argv);

    if (!module.configure(rf))
    {
        fprintf(stderr, "Error configuring module returning\n");
        return -1;
    }


    module.runModule();

    printf("Module shutting down\n");

    return 0;
}