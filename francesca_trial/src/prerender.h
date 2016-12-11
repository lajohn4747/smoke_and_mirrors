//
// Created by John La on 12/11/16.
//

#ifndef SIMULATION_PRERENDER_H
#define SIMULATION_PRERENDER_H

#endif //SIMULATION_PRERENDER_H

#include <vector>
#include "particlesystem.h"

class PrerenderSystem: public ParticleSystem{
public:
    PrerenderSystem();

    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void draw(GLProgram&);

    // inherits
    // std::vector<Vector3f> m_vVecState;
};