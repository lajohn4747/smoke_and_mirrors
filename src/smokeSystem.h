#ifndef SMOKE_H
#define SMOKE_H

#include <vector>

#include "particlesystem.h"

class SmokeSystem : public ParticleSystem
{
public:
    SmokeSystem();

    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void draw(GLProgram&);

    // inherits 
    // std::vector<Vector3f> m_vVecState;
private:
    Vector3f computeGravity(float mass);
    Vector3f computeDrag(float k, Vector3f velocity);
    Vector3f computeSpring(float k, float r, int i, int j, std::vector<Vector3f> pos);
    std::vector<Vector3f> getPositions(std::vector<Vector3f> state);
    std::vector<Vector3f> getVelocities(std::vector<Vector3f> state);
};

#endif
