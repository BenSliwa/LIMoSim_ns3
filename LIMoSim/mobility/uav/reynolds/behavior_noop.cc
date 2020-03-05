#include "behavior_noop.h"

namespace LIMoSim {

Behavior_NoOp::Behavior_NoOp():
    Behavior("NoOp")
{

}


Steering LIMoSim::Behavior_NoOp::apply()
{
    return Steering();
}

} // namespace LIMoSim

