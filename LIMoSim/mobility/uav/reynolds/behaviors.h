#ifndef BEHAVIORS_H
#define BEHAVIORS_H

#include "behavior.h"
#include "behavior_circlearound.h"
#include "behavior_arrive.h"
#include "behavior_noop.h"
#include "behavior_flee.h"
#include "behavior_seek.h"
#include "behavior_wander.h"
#include "behavior_pursuit.h"
#include "behavior_evasion.h"
#include "behavior_separation.h"
#include "behavior_cohesion.h"
#include "behavior_cohesion2d.h"
#include "behavior_idselectivecohesion2d.h"
#include "behavior_cohesion2dregionwise.h"
#include "behavior_flocking.h"
#include "behavior_collisionavoidance.h"
#include "behavior_composition.h"
#include "behavior_seekwithspeed.h"
#include "behavior_randomwalk.h"
#include "behavior_randomwaypoint.h"
#include "behavior_selfalignment.h"
#include "behavior_alignment.h"
#include "behavior_leaderfollowing.h"
#include "behavior_gaussmarkovrandomwalk.h"
#include "behavior_followatelevation.h"
#include "behavior_waypointroute.h"

namespace LIMoSim {

Behavior* makeSelfAligned(Behavior *_behavior, std::string _agentId = "");

}

#endif // BEHAVIORS_H
