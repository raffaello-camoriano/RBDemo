--
-- Copyright (C) 2014 IIT iCub Facility
-- Author: Tanis Mar
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--
 
-- loading lua-yarp binding library
require("yarp")

origin = {0,0,0}
thresh = 2

-- This monitor controls the transmision of info from the NearThingsDetector module to the gaze controller.
-- Therefore, it has two conditions. 
-- First, it accepts the data unless there is other data from the touch detector
-- Secondly, it checks that the object detected is not too far. If it is too far, it blocks the transmsion

--
-- create is called when the port monitor is created 
-- @return Boolean
--
PortMonitor.create = function()
    -- Accepts data unless a touch event happens.
    PortMonitor.setConstraint("not e_touch")
    return true;
end


--
-- accept is called when the port receives new data
-- @param thing The Things abstract data type
-- @return Boolean
-- if false is returned, the data will be ignored 
-- and update() will never be called
PortMonitor.accept = function(thing)
    -- Lets data pass to gaze controller only if the distance is under a threshold
    print("selector: PortMonitor.accept()")
    X = thing:asBottle():get(0):asDouble()
    Y = thing:asBottle():get(1):asDouble()
    Z = thing:asBottle():get(2):asDouble()
    
    dist3D = sqrt( (X-origin[0])^2+(Y-origin[1])^2+(Z-origin[2])^2)
    if (dist3D < thresh) then
        print(">>>>>>> Port monitor: Coordinates accepted, distance within limits")
        return true
    else
        print(">>>>>>> Port monitor: Coordinates blocked, object too far")
        return false
    end
    
end

