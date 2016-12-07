//
// Created by John La on 12/7/16.
//

#include "mirror.h"
#include "camera.h"
#include "vertexrecorder.h"

Mirror::Mirror(){
    m_vVecState.push_back(Vector3f(0.0f, -3.0f, 0.0));
};

// Particle system integrator for the ball -> Put all the forces in the vector
std::vector<Vector3f> Mirror::evalF(std::vector<Vector3f> state) {
    std::vector<Vector3f> f;
    return f;
}

void Mirror::draw(GLProgram &gl) {
    const Vector3f PARTICLE_COLOR(0.0f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
    Vector3f pos = getState()[0];
    gl.updateModelMatrix(Matrix4f::translation(pos));
    drawQuad(5);
}
//TODO Collision detection
bool Mirror::checkCollision() {
    return false;
}