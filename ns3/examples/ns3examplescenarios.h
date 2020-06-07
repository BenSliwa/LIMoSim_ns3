#ifndef NS3EXAMPLESCENARIOS_H
#define NS3EXAMPLESCENARIOS_H

namespace LIMoSim {
namespace NS3 {
namespace Examples {

void lteSimpleEpcMobile();
void lteSimpleEpcMobile_mult();
void lteSimpleEpcStatic();
void lteSimpleStatic();
void lteCoverageMobile();
void lteCoverageMobile_mult(unsigned int runs = 5);
void lteCoverageMobile_perf(unsigned int runs = 5);
void lteCoverageStatic();
void lteCoverageStatic_mult(unsigned int runs = 5);
void lteCoverageStatic_perf(unsigned int runs = 5);
void lteBeamSteering();
void lteBeamSteering_mult(unsigned int runs = 5);
void lteFollower();
void lteFollower_mult(unsigned int runs = 5);
void lteAerialBasestationCluster();
void lteAerialBasestationCluster_mult(unsigned int runs = 5);
#ifdef NS_MMW_BUILD
void mmwaveSimpleEpcStatic();
void mmwaveSimpleEpcStatic_mult(unsigned int runs = 1);
void mmwaveCarFollowing();
void mmwaveBeamSteering();
#endif

#ifdef NS_V2X_BUILD
void v2xSimple();
void v2xSimple_mult(unsigned int runs = 5);
#endif
void wifiReachMobile();
void wifiReachStatic();

}
namespace helpers {
void loadScenarioRegistry();
void printAvailableScenarios();

} // namespace helpers
}
}

#endif // NS3EXAMPLESCENARIOS_H
