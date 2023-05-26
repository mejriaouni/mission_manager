#ifndef _MISSIONENUM_H_
#define _MISSIONENUM_H_

namespace done
{

enum class Mission
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

}//namespace done

#endif
