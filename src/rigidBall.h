//
// Created by John La on 11/26/16.
//
#pragma once
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

    bool isColliding = false;
    Vector3f extraForces = Vector3f::ZERO;
    Vector3f getCenter();
    float radius;
    float getRadius();
    Vector3f center;

private:
    // Gravity on the ball
    Vector3f computeGravity(float mass);
    Vector3f computeDrag(float k, Vector3f velocity);
};