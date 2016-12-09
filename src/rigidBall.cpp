//
// Created by John La on 11/26/16.
//

#include "rigidBall.h"
#include "camera.h"
#include "vertexrecorder.h"

// Initialize the ball with a start state
RigidBall::RigidBall() {
    center = Vector3f(0.0f, 4.0f, 0.0);
    m_vVecState.push_back(Vector3f(0.0f, 0.0f, 0.0f));
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
    Vector3f allForces;
    Vector3f velocity = getState()[0];
    float drag = 0.11f;
    f.push_back(velocity);
    allForces = computeGravity(0.1);
    if (isColliding) {
        allForces += extraForces;
    }
    allForces += computeDrag(drag, velocity);
    f.push_back(allForces);
    return f;
}

Vector3f RigidBall::computeDrag(float k, Vector3f velocity){
    return -k * velocity;
}

float RigidBall::getRadius() {
    return radius;
}

Vector3f RigidBall::getCenter() {
    return center;
}

void RigidBall::draw(GLProgram &gl) {
    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
    Vector3f pos = getState()[1];
    center = pos;
    gl.updateModelMatrix(Matrix4f::translation(pos));
    drawSphere(radius, 30, 10);
}
//TODO Collision detection
bool RigidBall::checkCollision() {
    return false;
}