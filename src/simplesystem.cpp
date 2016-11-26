#include "simplesystem.h"

#include "camera.h"
#include "vertexrecorder.h"


SimpleSystem::SimpleSystem()
{
    // 3.2 initialize the simple system
    // Choosing a state on the circle that is valid
    m_vVecState.push_back(Vector3f(1.0f, 0.0, 0.0));

}

std::vector<Vector3f> SimpleSystem::evalF(std::vector<Vector3f> state)
{

    // 3.2: implement evalF
    // for a given state, evaluate f(X,t)
    std::vector<Vector3f> velocity;
    for(unsigned i = 0; i<state.size(); ++i){
        Vector3f derivV = Vector3f(-1.0*state[i][1], state[i][0], 0.0);
        velocity.push_back(derivV);
    }

    return velocity;
}

// render the system (ie draw the particles)
void SimpleSystem::draw(GLProgram& gl)
{

    // draw the particle.
    //           we provide code that draws a static sphere.
    //           you should replace it with your own
    //           drawing code.
    //           In this assignment, you must manage two
    //           kinds of uniforms before you draw
    //            1. Update material uniforms (color)
    //            2. Update transform uniforms
    //           GLProgram is a helper object that has
    //           methods to set the uniform state.

    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
    //Vector3f pos(1, 0, 0); //YOUR PARTICLE POSITION
    Vector3f pos = getState()[0];
    gl.updateModelMatrix(Matrix4f::translation(pos));
    drawSphere(0.075f, 10, 10);


}
