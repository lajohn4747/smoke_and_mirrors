#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vector>

#include "particlesystem.h"

class PendulumSystem : public ParticleSystem
{
public:
    PendulumSystem();

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
