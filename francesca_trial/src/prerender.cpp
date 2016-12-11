//
// Created by John La on 12/11/16.
//
#include "prerender.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

const int NUM_PARTICLES = 30;
string PATH = "positions.txt";

PrerenderSystem::PrerenderSystem() {
    string line;
    ifstream myFile(PATH);
    m_vVecState.push_back(Vector3f(0,0,0));
    if (myFile.is_open()){
        while (getline(myFile, line)){
            float x, y, z;
            string currentLine = line.substr(1, line.size()-2);
            currentLine.erase( std::remove( currentLine.begin(), currentLine.end(), ',' ), currentLine.end() ) ;
            //printf("%s This is \n", currentLine.c_str());
            std::istringstream iss(currentLine);

            iss >> x >> y >> z;
            Vector3f pos(x,y,z);

            allPositions.push_back(pos);
            //pos.print();

        }
    }

    lastIteration = allPositions.size() / NUM_PARTICLES;

}

vector<Vector3f> PrerenderSystem::evalF(std::vector<Vector3f> state) {
    vector<Vector3f> f;
    f.push_back(Vector3f(0,0,0));
    return f;
}

void PrerenderSystem::draw(GLProgram& gl) {
    //getState()[0].print();
    //printf("%lu \n", allPositions.size());
    //printf("%lu\n", lastIteration);
    if(currentIteration < lastIteration) {
        const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
        gl.updateMaterial(PENDULUM_COLOR);

        for (unsigned int i = 0; i < NUM_PARTICLES; ++i) {
            gl.updateModelMatrix(Matrix4f::translation(allPositions[NUM_PARTICLES * currentIteration + i]));
            printf("%d\n", NUM_PARTICLES * currentIteration + i);
            gl.updateMaterial(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(-1, -1, -1), Vector3f(0, 0, 0), 1.0f, 0.5f);
            drawSphere(0.05f, 8, 8);
        }
    }

    currentIteration += 1;
}
