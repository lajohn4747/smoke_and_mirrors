#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include <vector>

#include "particlesystem.h"

class ClothSystem : public ParticleSystem
{
    ///ADD MORE FUNCTION AND FIELDS HERE
public:
    ClothSystem();

    // evalF is called by the integrator at least once per time step
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;

    // draw is called once per frame
    void draw(GLProgram& ctx);

    //get the index of the position of the particle in a state
    int indexOf(int i, int j);

    // inherits
    // std::vector<Vector3f> m_vVecState;
  protected:
    //syntax of a spring Vector4f: [index of particle 1 position, index of particle 2 position, rest lenght, stiffness]
    std::vector<Vector4f> m_structuralSprings;
    std::vector<Vector4f> m_shearSprings;
    std::vector<Vector4f> m_flexionSprings;
};


#endif