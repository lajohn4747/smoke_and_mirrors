#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

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

    // draw is called once per frame
    void draw(GLProgram& ctx);

    //get the index of the position of the particle in a state
    int indexOf(int i, int j);

    std::vector<Vector3f> extraForces;

    // inherits
    // std::vector<Vector3f> m_vVecState;

    //checks if there are particles colliding with the cloth.
    //If there are, return a list of those particles's positions in the state of the cloth system
    //If not, return an empty list
    std::vector<int> pointsCollidingWithBall(RigidBall* ball);

    //gets the summed force to apply to the Ball in the next time step.
    //Given a list of the indexes of the particles affecting the ball in the cloth system (returned by pointsCollidingWithBall)
    //If no force should be applied to the ball, return (0,0,0)
    Vector3f getForceOnBall(std::vector<int> particlesTouchingBall);
    
    //gets a list of forces to apply to each of the particles in the cloth system in the next time step.
    //returns a list of length m_vVecState/2, with a force for each particle in the system dur to the ball.
    //if there is no force added to a certain particle, let the force at that index in the returned vector be (0,0,0)
    std::vector<Vector3f> getForcesOnPartices(std::vector<int> particlesTouchingBall, RigidBall* ball);
  protected:
    //syntax of a spring Vector4f: [index of particle 1 position, index of particle 2 position, rest lenght, stiffness]
    std::vector<Vector4f> m_structuralSprings;
    std::vector<Vector4f> m_shearSprings;
    std::vector<Vector4f> m_flexionSprings;
};


#endif
