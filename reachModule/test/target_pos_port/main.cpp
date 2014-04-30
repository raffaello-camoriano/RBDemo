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

using namespace std;
using namespace yarp::os;



int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    
    BufferedPort<Bottle> p;            // Create a port.
    p.open("/test_out");    // Give it a name on the network.
    while (true) {
        Bottle& b = p.prepare();        // Make a place to store things.
        b.addDouble(-0.3);
        b.addDouble(0.1);
        b.addDouble(0.05);
        p.write();      // Send the data.
    }

    Network::connect("/test_out","/reachingTarget:i");  // connect two ports.
}