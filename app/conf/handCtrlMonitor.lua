--
-- Copyright (C) 2012 IITRBCS
-- Author: Raffaello Camoriano
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--

-- How to use the portMonitor

-- In general:
--  $ yarp connect /out /in tcp+recv.portmonitor+script.lua+context.myapp+file.my_lua_script
  
--  'script.lua' tells the carrier to create a port monitor object for Lua.  
--  'context.myapp' tells the resource finder to load the script from the 'myapp' context. 
-- 'file.my_lua_script' indicates 'my_lua_script' should be loaded by monitor object. 
--  'my_lua_script' is located using standard yarp Resource Finder policy. The postfix 
--  (e.g., '.lua') is not necessary.

-- In our case:
-- yarp connect /reachModule/handToBeClosed:o /handCtrl/handToBeClosed:i tcp+recv.portmonitor+script.lua+context.RBDemo+file.handCtrlMonitor
 
-- yarp connect /touchDetector/contact_pos:o /handCtrl/handToBeClosed:i tcp+recv.portmonitor+script.lua+context.RBDemo+file.handCtrlMonitor


 
-- loading lua-yarp binding library
require("yarp")



--
-- create is called when the port monitor is created 
-- @return Boolean
--
PortMonitor.create = function()

    return true;
end



-- 
-- destroy is called when port monitor is destroyed
--
PortMonitor.destroy = function()
end



--
-- accept is called when the port receives new data
-- @param thing The Things abstract data type
-- @return Boolean
-- if false is returned, the data will be ignored 
-- and update() will never be called
PortMonitor.accept = function(thing)
    print("selector: PortMonitor.accept()")
    if (thing:asBottle():toInt() == 1 or thing:asBottle():toInt() == 4) then
        return true
    else
        return false
    end
end

