//
// Created by John La on 11/26/16.
//

#include "rigidBall.h"
#include "camera.h"
#include "vertexrecorder.h"

// Initialize the ball with a start state
RigidBall::RigidBall() {
    m_vVecState.push_back(Vector3f(0.0f, 4.0f, 0.0));
    radius = 0.25f;
}

// Get the gravity force of the rigid ball
Vector3f RigidBall::computeGravity(float mass) {
    return mass * Vector3f(0, -9.8, 0);
}

// Particle system integrator for the ball -> Put all the forces in the vector
std::vector<Vector3f> RigidBall::evalF(std::vector<Vector3f> state) {
    std::vector<Vector3f> f;
    f.push_back(computeGravity(0.1));
    return f;
}

float RigidBall::getRadius() {
    return radius;
}

Vector3f RigidBall::getCenter() {
    return getState()[0];
}

void RigidBall::draw(GLProgram &gl) {
    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
    Vector3f pos = getState()[0];
    gl.updateModelMatrix(Matrix4f::translation(pos));
    drawSphere(radius, 30, 10);
}
//TODO Collision detection
bool RigidBall::checkCollision() {
    return false;
}