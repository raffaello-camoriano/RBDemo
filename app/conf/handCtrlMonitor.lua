--
-- Copyright (C) 2012 IITRBCS
-- Author: Raffaello Camoriano
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--
 
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
    if thing.get(0) == 1 || thing.get(0) == 4 then
        return true
    else
        return false
    end
end

