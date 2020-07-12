#include "Configuration.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <sstream>

Configuration::Configuration() : start(3, 0), goal(3, 0), boundary(6, 0) {}

bool Configuration::Load(const string& file) {
    ifstream inFile(file.c_str());

    if (!inFile.good()) {
        cout << "Cannot load file, Please try again... " << endl;
        exit(0);
    }

    while (inFile.good() && !inFile.eof()) {
        string line;
        getline(inFile, line);

        int pos = line.find('=');
        if (pos != string::npos) {
            string key = Trim(line.substr(0, pos));
            string value = Trim(line.substr(pos + 1));

            stringstream ss(value);
            istream_iterator<string> begin(ss);
            istream_iterator<string> end;
            vector<string> vstrings(begin, end);

            if (key == "time") {
                time = atof(value.c_str());
            } else if (key == "start") {
                transform(vstrings.begin(), vstrings.end(), start.begin(),
                          [](const string& val) { return std::stof(val); });
            } else if (key == "goal") {
                transform(vstrings.begin(), vstrings.end(), goal.begin(),
                          [](const string& val) { return std::stof(val); });
            } else if (key == "boundary") {
                transform(vstrings.begin(), vstrings.end(), boundary.begin(),
                          [](const string& val) { return std::stof(val); });
            } else {
                vector<float> block(vstrings.size());
                transform(vstrings.begin(), vstrings.end(), block.begin(),
                          [](const string& val) { return std::stof(val); });
                blocks.push_back(block);
            }
        }
    }

    return true;
}

float Configuration::getTime() { return time; }

vector<float> Configuration::getStart() { return start; }

vector<float> Configuration::getGoal() { return goal; }

vector<float> Configuration::getBoundary() { return boundary; }

vector<vector<float>> Configuration::getBlocks() { return blocks; }

string Configuration::Trim(const string& str) {
    int first = str.find_first_not_of(" \t");

    if (first != string::npos) {
        int last = str.find_last_not_of(" \t");

        return str.substr(first, last - first + 1);
    } else {
        return "";
    }
}