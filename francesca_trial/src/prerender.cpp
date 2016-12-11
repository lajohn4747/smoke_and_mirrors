//
// Created by John La on 12/11/16.
//
#include "prerender.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

string PATH;

PrerenderSystem::PrerenderSystem() {
    string line;
    ifstream myFile(PATH);
    if (myFile.is_open()){
        while (getline(myFile, line)){
            printf("%s\n", line.c_str());
        }
    }

}

vector<Vector3f> PrerenderSystem::evalF(std::vector<Vector3f> state) {
    vector<Vector3f> f;
    f.push_back(Vector3f(0,0,0));
    return f;
}
