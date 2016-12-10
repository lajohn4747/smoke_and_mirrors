#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vector>

#include "particlesystem.h"

class PendulumSystem : public ParticleSystem
{
public:
    PendulumSystem();

    //kernel function calculations
    float calcW(Vector3f r);
    Vector3f calcGradientW(Vector3f r);
    float calcLaplacianW(Vector3f r);

    float calculateDensity(std::vector<Vector3f> state, int x_i_index);
    Vector3f calculatePressureForce(std::vector<Vector3f> state, int x_i_index);
    Vector3f calculateViscosityForce(std::vector<Vector3f> state, int x_i_index);

    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void draw(GLProgram&);

    // inherits 
    // std::vector<Vector3f> m_vVecState;
 protected:
    //syntax of a spring Vector4f: [index of particle 1 position, index of particle 2 position, rest lenght, stiffness]
    std::vector<Vector4f> m_springs;
};

#endif
