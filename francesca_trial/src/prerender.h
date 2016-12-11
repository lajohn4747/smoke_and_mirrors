//
// Created by John La on 12/11/16.
//

#ifndef SIMULATION_PRERENDER_H
#define SIMULATION_PRERENDER_H

#endif //SIMULATION_PRERENDER_H

#include <vector>
#include "particlesystem.h"
#include <string>

class PrerenderSystem: public ParticleSystem{
public:
    PrerenderSystem();

    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void draw(GLProgram&);

    int currentIterationSmoke = 0;
    int currentIterationCloth = 0;
    int currentIterationBall = 0;
    int lastIterationSmoke;
    int lastIterationCloth;
    int lastIterationBall;
    std::vector<Vector3f> allPositionsSmoke;
    std::vector<Vector3f> allPositionsBall;
    std::vector<Vector3f> allPositionsCloth;
    // inherits
    // std::vector<Vector3f> m_vVecState;
private:
    void removeCharsFromString(std::string &str, std::string charsToRemove);
};