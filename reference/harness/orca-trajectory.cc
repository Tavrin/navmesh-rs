// Public-API trajectory collector. Initial positions/goals come from the shared
// fixture exporter; output is consumed by the same metric collector as the port.
#include <RVO.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
int main() {
  std::string scene; size_t count, ticks;
  if (!(std::cin >> scene >> count >> ticks)) return 1;
  RVO::RVOSimulator sim(.05f, 8.f, 16, 2.f, 1.f, .4f, 2.f);
  std::vector<RVO::Vector2> goals;
  for (size_t i=0;i<count;++i) {
    float x,z,gx,gz; if (!(std::cin>>x>>z>>gx>>gz)) return 2;
    sim.addAgent(RVO::Vector2(x,z)); goals.emplace_back(gx,gz);
  }
  auto wall = [&](float x,float z,float ex,float ez) {
    sim.addObstacle({RVO::Vector2(x,z),RVO::Vector2(ex,ez)});
  };
  float height=scene=="corridor"?8.f:240.f;
  wall(0,0,240,0); wall(240,0,240,height);
  wall(240,height,0,height); wall(0,height,0,0);
  if(scene=="doorway") {
    wall(118,0,118,118); wall(118,122,118,240);
    wall(122,0,122,118); wall(122,122,122,240);
    wall(118,118,122,118); wall(118,122,122,122);
  }
  sim.processObstacles();
  std::cout<<std::setprecision(9);
  for(size_t t=0;t<ticks;++t) {
    for(size_t i=0;i<count;++i) {
      auto pos=sim.getAgentPosition(i);
      // A waypoint is needed to traverse the concave doorway. This is explicit
      // reference navigation policy; it is not the port's corridor funnel.
      auto goal=scene=="doorway" && pos.x()<122.f?RVO::Vector2(122.f,120.f):goals[i];
      auto delta=goal-pos; float distance=RVO::abs(delta);
      float speed=std::min(distance/.05f,2.f);
      sim.setAgentPrefVelocity(i,distance>.001f?RVO::Vector2(delta.x()/distance*speed,delta.y()/distance*speed):RVO::Vector2());
    }
    sim.doStep();
    for(size_t i=0;i<count;++i) {
      auto p=sim.getAgentPosition(i); auto v=sim.getAgentVelocity(i);
      std::cout<<p.x()<<' '<<p.y()<<' '<<v.x()<<' '<<v.y()<<'\n';
    }
  }
  return 0;
}
