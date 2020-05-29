#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <iostream>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RRT_Planner {
   private:
    float time;
    std::vector<float> start_coord;
    std::vector<float> goal_coord;
    std::vector<float> boundary;
    std::vector<std::vector<float>> blocks;

   public:
    RRT_Planner(float time, std::vector<float> start_coord, std::vector<float> goal_coord,
                std::vector<float> boundary, std::vector<std::vector<float>> blocks);

    // state validity checking function
    bool isStateValid(const ob::State *state);

    // planner
    void plan();
};

RRT_Planner::RRT_Planner(float time, std::vector<float> start_coord, std::vector<float> goal_coord,
                         std::vector<float> boundary, std::vector<std::vector<float>> blocks)
    : time(time),
      start_coord(start_coord),
      goal_coord(goal_coord),
      boundary(boundary),
      blocks(blocks) {}

bool RRT_Planner::isStateValid(const ob::State *state) {
    // cast the abstract state type to the type we expect
    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

    float x = pos->values[0];
    float y = pos->values[1];
    float z = pos->values[2];

    // check validity of state
    for (auto block : blocks) {
        float x_min = block[0];
        float y_min = block[1];
        float z_min = block[2];
        float x_max = block[3];
        float y_max = block[4];
        float z_max = block[5];

        if (x >= x_min - 0.1 && x <= x_max + 0.1 && y >= y_min - 0.1 && y <= y_max + 0.1 &&
            z >= z_min - 0.1 && z <= z_max + 0.1) {
            return false;
        }
    }
    return true;
}

void RRT_Planner::plan() {
    // construct a state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // set the bounds for the space
    float x_min = boundary[0];
    float y_min = boundary[1];
    float z_min = boundary[2];
    float x_max = boundary[3];
    float y_max = boundary[4];
    float z_max = boundary[5];
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, x_min);
    bounds.setLow(1, y_min);
    bounds.setLow(2, z_min);
    bounds.setHigh(0, x_max);
    bounds.setHigh(1, y_max);
    bounds.setHigh(2, z_max);
    space->setBounds(bounds);

    // crate start state and goal state
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    start[0] = start_coord[0];
    start[1] = start_coord[1];
    start[2] = start_coord[2];
    goal[0] = goal_coord[0];
    goal[1] = goal_coord[1];
    goal[2] = goal_coord[2];

    // define space information for this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(std::bind(&RRT_Planner::isStateValid, this, std::placeholders::_1));
    si->setStateValidityCheckingResolution(0.01);
    si->setup();

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);

    // define a RRT planner
    auto planner(std::make_shared<og::RRT>(si));
    planner->setProblemDefinition(pdef);  // set the problem we are trying to solve for the planner
    planner->setup();

    // this call is optional, but we put it in to get more output information
    pdef->print(std::cout);

    // solve the problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(time);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        pdef->getSolutionPath()->print(std::cout);
    } else {
        std::cout << "No solution found" << std::endl;
    }
}

int main(int argc, const char *argv[]) {
    float time = 5.0;
    std::vector<float> start_coord{2.3, 2.3, 1.3};
    std::vector<float> goal_coord{7.0, 7.0, 5.5};
    std::vector<float> boundary{-5, -5, -5, 10, 10, 10};
    std::vector<std::vector<float>> blocks{{4.5, 4.5, 2.5, 5.5, 5.5, 3.5}};

    RRT_Planner rrt(time, start_coord, goal_coord, boundary, blocks);
    rrt.plan();

    return 0;
}