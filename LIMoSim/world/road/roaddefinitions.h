#ifndef LIMOSIM_ROADDEFINITIONS_H
#define LIMOSIM_ROADDEFINITIONS_H

namespace LIMoSim
{
    namespace LINK_TYPE
    {
        enum{
            INVALID,
            END_TO_START,
            END_TO_END,
            START_TO_START,
            START_TO_END
        };
    }

    namespace ROAD_TYPE
    {
        enum{
            MOTORWAY,
            MOTORWAY_LINK,
            MOTORWAY_JUNCTION,
            TRUNK,
            TRUNK_LINK,
            PRIMARY,
            SECONDARY,
            TERTIARY,
            RESIDENTIAL,
            SERVICE
        };
    }

    namespace TRANSITION_TYPE
    {
        enum{
            EQUAL,
            INFERIOR,
            SUPERIOR
        };
    }
}





#endif // LIMOSIM_ROADDEFINITIONS_H
