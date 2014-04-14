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





int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)
    
    myReport rep;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setMonitor(&rep);
    rf.setDefaultContext("demoGrasp_IIT_ISR");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    managerModule mod;
    mod.setName("/demoGraspManager_IIT_ISR");

    return mod.runModule(rf);
}
