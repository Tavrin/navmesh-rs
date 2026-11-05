#include <RVO.h>
#include <iostream>
#include <iomanip>
#include <vector>
// Public API only. Each case advances once from the supplied frozen state.
int main() {
  std::cout << std::setprecision(9);
  size_t n, m; float dt, horizon, obst; size_t cap;
  while (std::cin >> n >> m >> dt >> horizon >> obst >> cap) {
    RVO::RVOSimulator sim(dt, 1e6f, cap, horizon, obst, .5f, 2.f);
    for (size_t i=0; i<n; ++i) {
      float x,y,vx,vy,px,py,r,s;
      std::cin >> x >> y >> vx >> vy >> px >> py >> r >> s;
      sim.addAgent(RVO::Vector2(x,y));
      sim.setAgentRadius(i,r); sim.setAgentMaxSpeed(i,s);
      sim.setAgentVelocity(i,RVO::Vector2(vx,vy));
      sim.setAgentPrefVelocity(i,RVO::Vector2(px,py));
    }
    for (size_t i=0;i<m;++i) {
      float ax,ay,bx,by; std::cin >> ax >> ay >> bx >> by;
      std::vector<RVO::Vector2> wall;
      wall.push_back(RVO::Vector2(ax,ay)); wall.push_back(RVO::Vector2(bx,by));
      sim.addObstacle(wall);
    }
    sim.processObstacles(); sim.doStep();
    for (size_t i=0;i<n;++i) {
      const RVO::Vector2 v=sim.getAgentVelocity(i);
      std::cout << v.x() << ' ' << v.y() << '\n';
    }
  }
}
