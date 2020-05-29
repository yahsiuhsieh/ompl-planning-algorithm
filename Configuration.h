#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <fstream>
#include <string>
#include <vector>

using namespace std;

class Configuration {
   public:
    // class constructor
    Configuration();

    // load a configuration file
    bool Load(const string& File);

    // remove leading and trailing tabs and spaces
    string Trim(const string& str);

    // get data
    float getTime();
    vector<float> getStart();
    vector<float> getGoal();
    vector<float> getBoundary();
    vector<vector<float>> getBlocks();

   private:
    // the container
    float time;
    vector<float> start;
    vector<float> goal;
    vector<float> boundary;
    vector<vector<float>> blocks;
};
#endif
