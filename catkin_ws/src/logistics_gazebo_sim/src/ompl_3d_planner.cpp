#include <algorithm>
#include <cmath>
#include <ompl/base/ScopedState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Obstacle {
  bool cylinder;
  double x, y, a, b, height;
};

int main(int argc, char **argv) {
  if (argc != 12) {
    std::cerr << "usage: ompl_3d_planner sx sy sz gx gy gz obstacles.csv output.csv z_low z_high solve_seconds\n";
    return 2;
  }
  const double sx=std::stod(argv[1]), sy=std::stod(argv[2]), sz=std::stod(argv[3]);
  const double gx=std::stod(argv[4]), gy=std::stod(argv[5]), gz=std::stod(argv[6]);
  const double solve_seconds=std::stod(argv[11]);
  std::vector<Obstacle> obstacles;
  std::ifstream input(argv[7]);
  std::string line;
  while (std::getline(input,line)) {
    if (line.empty()) continue;
    std::replace(line.begin(),line.end(),',',' ');
    std::istringstream row(line);
    std::string kind; Obstacle o{};
    row>>kind>>o.x>>o.y>>o.a>>o.b>>o.height;
    o.cylinder=(kind=="cylinder");
    if (row) obstacles.push_back(o);
  }

  auto space=std::make_shared<ob::RealVectorStateSpace>(3);
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0,-46.0);bounds.setHigh(0,46.0);
  bounds.setLow(1,-46.0);bounds.setHigh(1,46.0);
  bounds.setLow(2,std::stod(argv[9]));bounds.setHigh(2,std::stod(argv[10]));
  space->setBounds(bounds);
  og::SimpleSetup setup(space);
  setup.setStateValidityChecker([&](const ob::State *state) {
    const auto *v=state->as<ob::RealVectorStateSpace::StateType>();
    const double x=(*v)[0],y=(*v)[1],z=(*v)[2];
    for (const auto &o:obstacles) {
      if (z>o.height) continue;
      if (o.cylinder) {
        const double dx=x-o.x,dy=y-o.y;
        if (dx*dx+dy*dy<=o.a*o.a) return false;
      } else if (std::abs(x-o.x)<=o.a && std::abs(y-o.y)<=o.b) return false;
    }
    return true;
  });
  setup.getSpaceInformation()->setStateValidityCheckingResolution(0.003);
  ob::ScopedState<> start(space),goal(space);
  start[0]=sx;start[1]=sy;start[2]=sz;
  goal[0]=gx;goal[1]=gy;goal[2]=gz;
  setup.setStartAndGoalStates(start,goal,0.2);
  auto planner=std::make_shared<og::InformedRRTstar>(setup.getSpaceInformation());
  planner->setRange(4.0);
  planner->setGoalBias(0.08);
  setup.setPlanner(planner);
  setup.setOptimizationObjective(
      std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
  const auto result=setup.solve(solve_seconds);
  if (!result) {
    std::cerr<<"E_OMPL_NO_PATH\n";
    return 3;
  }
  auto &path=setup.getSolutionPath();
  og::PathSimplifier simplifier(setup.getSpaceInformation());
  simplifier.reduceVertices(path);
  simplifier.shortcutPath(path);
  simplifier.smoothBSpline(path,3,0.03);
  const unsigned count=std::max<unsigned>(4,static_cast<unsigned>(std::ceil(path.length()/0.75))+1);
  path.interpolate(count);
  std::ofstream output(argv[8]);
  output<<std::fixed<<std::setprecision(6);
  for (const auto *state:path.getStates()) {
    const auto *v=state->as<ob::RealVectorStateSpace::StateType>();
    output<<(*v)[0]<<","<<(*v)[1]<<","<<(*v)[2]<<"\n";
  }
  std::cout<<"OMPL_OK states="<<path.getStateCount()<<" length="<<path.length()<<"\n";
  return 0;
}
