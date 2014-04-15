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

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;

class reachingRF: public RFModule
{
protected:

    
public:
    reachingRF() {};
    
    bool updateModule() { return true; }
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

    reachingRF mod;
    
    cout << "Configuring and starting reaching module. \n";
    return mod.runModule(rf);
}
