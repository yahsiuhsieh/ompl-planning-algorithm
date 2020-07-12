#ifndef PLANNER_H
#define PLANNER_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner {
   private:
    float time;
    std::vector<float> start_coord;
    std::vector<float> goal_coord;
    std::vector<float> boundary;
    std::vector<std::vector<float>> blocks;

   public:
    Planner(float time, std::vector<float> start_coord, std::vector<float> goal_coord,
            std::vector<float> boundary, std::vector<std::vector<float>> blocks);

    // state validity checking function
    bool isStateValid(const ob::State *state);

    // planner
    bool plan();
};
#endif
