#include (./mmwave/Mmwave.pri)
equals(ns3, mmw) {
    include (./mmwave-off/Mmwave.pri)
}
include (deterministic-obstacle-shadowing/DeterministicObstacleShadowing.pri)
include (./net-stats/NetStats.pri)
