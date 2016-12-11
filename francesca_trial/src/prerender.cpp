//
// Created by John La on 12/11/16.
//
#include "prerender.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

const int NUM_PARTICLES = 30;
const int CLOTH_PARTICLES = 400;
string PATH = "positions.txt";

PrerenderSystem::PrerenderSystem() {
    string line;
    ifstream myFile(PATH);
    m_vVecState.push_back(Vector3f(0,0,0));
    if (myFile.is_open()){
        while (getline(myFile, line)){
            float x, y, z;
            //string currentLine = line.substr(1, line.size()-2);
            string currentLine = line;
            string typeParticle;
            removeCharsFromString(currentLine, "<>");
            currentLine.erase( std::remove( currentLine.begin(), currentLine.end(), ',' ), currentLine.end() ) ;
            //printf("%s This is \n", currentLine.c_str());
            std::istringstream iss(currentLine);

            iss >> typeParticle >> x >> y >> z;
            Vector3f pos(x,y,z);
            if (typeParticle == "S") {
                allPositionsSmoke.push_back(pos);
            } else if (typeParticle == "C"){
                allPositionsCloth.push_back(pos);
            } else if (typeParticle == "B"){
                allPositionsBall.push_back(pos);
            }


            //pos.print();

        }
    }

    lastIterationBall = allPositionsBall.size();
    lastIterationSmoke = allPositionsSmoke.size() / NUM_PARTICLES;
    lastIterationCloth = allPositionsCloth.size() / CLOTH_PARTICLES;
}

void PrerenderSystem::removeCharsFromString( string &str, string charsToRemove ) {
    for ( unsigned int i = 0; i < charsToRemove.size(); ++i ) {
        str.erase( remove(str.begin(), str.end(), charsToRemove[i]), str.end() );
    }
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
    if(currentIterationSmoke < lastIterationSmoke) {
        const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
        gl.updateMaterial(PENDULUM_COLOR);

        for (unsigned int i = 0; i < NUM_PARTICLES; ++i) {
            gl.updateModelMatrix(Matrix4f::translation(allPositionsSmoke[NUM_PARTICLES * currentIterationSmoke + i]));
            gl.updateMaterial(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(-1, -1, -1), Vector3f(0, 0, 0), 1.0f, 0.5f);
            drawSphere(0.05f, 8, 8);
        }
    }

    if(currentIterationCloth < lastIterationCloth){
        const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
        gl.updateMaterial(CLOTH_COLOR);

        /*for (unsigned int j = 0; j < CLOTH_PARTICLES; j++){
            gl.updateModelMatrix(Matrix4f::translation(allPositionsCloth[j + CLOTH_PARTICLES*currentIterationCloth]));
            drawSphere(0.04f, 8, 8);
        }*/
        gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
        VertexRecorder rec;
        for(unsigned k = 1; k < CLOTH_PARTICLES - 20; ++k){
            if(k % 20 != 0){
                Vector3f p1 = allPositionsCloth[k-1 + CLOTH_PARTICLES*currentIterationCloth];
                Vector3f p2 = allPositionsCloth[k + CLOTH_PARTICLES*currentIterationCloth];
                Vector3f p3 = allPositionsCloth[k+20-1 + CLOTH_PARTICLES*currentIterationCloth];
                Vector3f p4 = allPositionsCloth[k+20 + CLOTH_PARTICLES*currentIterationCloth];
                Vector3f N1 = Vector3f::cross( p3 - p1, p2 - p1);
                const Vector3f N(0, 1, 0);
                rec.record(p1, N);
                rec.record(p2, N);
                rec.record(p3, N);

                Vector3f N2 = Vector3f::cross(p3 - p2, p4 - p2);
                rec.record(p2, N);
                rec.record(p3, N);
                rec.record(p4, N);

                rec.draw();
            }
        }
    }

    if(currentIterationBall < lastIterationBall){
        gl.updateModelMatrix(Matrix4f::translation(allPositionsBall[currentIterationBall]));
        drawSphere(0.25f, 20, 20);
    }

    currentIterationSmoke += 1;
    currentIterationCloth += 1;
    currentIterationBall += 1;
}
