#pragma once

#include<ros/ros.h>

// Enumeration for mission and command types
// enum class MissionType {
//             charging, 
//             disinfection};

// enum class CommandType {
//             deleteMission, 
//             emergencyStop, 
//             cancelAllMissions, 
//             getStatus};



enum class MissionType : uint64_t
{
    NOMISSION         = 0,
    EMERGENCYSTOP_ON  = 1,
    EMERGENCYSTOP_OFF = 2,
    MISSION_PAUSE     = 3,
    MISSION_RESUME    = 4,
    DISINFECTROOM     = 5,
    BATTERYCHARGING   = 6,
    MISSION_STOP      = 7
};


// Error message types
// TODO

