#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H
#pragma once
#include <vector>

#include "particlesystem.h"
#include "rigidBall.h"

class ClothSystem : public ParticleSystem
{
    ///ADD MORE FUNCTION AND FIELDS HERE
public:
    ClothSystem();

    // evalF is called by the integrator at least once per time step
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;

    //clothBall Reference

    // draw is called once per frame
    void draw(GLProgram& ctx);

private:
    // Collisions
    bool checkCollisions(Vector3f pos);
    Vector3f computeGravity(float mass);
    Vector3f computeDrag(float k, Vector3f velocity);
    Vector3f computeSpring(float k, float r, int icol, int irow, int jcol, int jrow, std::vector<Vector3f> pos);
    std::vector<Vector3f> getPositions(std::vector<Vector3f> state);
    std::vector<Vector3f> getVelocities(std::vector<Vector3f> state);
    Vector3f pickVel(int row, int col, std::vector<Vector3f> velocities);
    Vector3f pickPos(int row, int col, std::vector<Vector3f> positions);
    Vector3f computeStructString(int i, int j, const std::vector<Vector3f> &pos);
    Vector3f computeShearString(int i, int j, const std::vector<Vector3f> &pos);
    Vector3f computeFlexString(int i, int j, const std::vector<Vector3f> &pos);
    // inherits
    // std::vector<Vector3f> m_vVecState;
};


#endif
