//
// Created by John La on 12/7/16.
//

#ifndef SIMULATION_MIRROR_H
#define SIMULATION_MIRROR_H

#endif //SIMULATION_MIRROR_H


#include "particlesystem.h"


class Mirror : public ParticleSystem
{
public:
    Mirror();

    // evalF is called by the integrator at least once per time step
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    // draw is called once per frame
    void draw(GLProgram& ctx);
    // Check if collision occurs
    bool checkCollision();

private:

};