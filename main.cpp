#include <iostream>
#include <vector>

#include "Configuration.h"
#include "Planner.h"

int main(int argc, const char *argv[]) {
    if (argc < 2) {
        std::cout << "Please specify configuration file..." << std::endl;
        exit(0);
    }

    Configuration config;
    config.Load(argv[1]);

    float time = config.getTime();
    std::vector<float> start_coord = config.getStart();
    std::vector<float> goal_coord = config.getGoal();
    std::vector<float> boundary = config.getBoundary();
    std::vector<std::vector<float>> blocks = config.getBlocks();

    Planner rrt(time, start_coord, goal_coord, boundary, blocks);
    rrt.plan();

    return 0;
}