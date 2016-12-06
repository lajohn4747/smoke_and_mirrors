//
// Created by John La on 11/26/16.
//

#ifndef SIMULATION_RIGIDBALL_H
#define SIMULATION_RIGIDBALL_H

#endif //SIMULATION_RIGIDBALL_H

#include "particlesystem.h"


class RigidBall : public ParticleSystem{
public:
    RigidBall();

    // evalF is called by the integrator at least once per time step
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    // draw is called once per frame
    void draw(GLProgram& ctx);
    // Check if collision occurs
    bool checkCollision();

private:
    // Gravity on the ball
    Vector3f computeGravity(float mass);
};