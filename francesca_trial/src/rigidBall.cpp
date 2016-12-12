//
// Created by John La on 11/26/16.
//

#include "rigidBall.h"
#include "camera.h"
#include "vertexrecorder.h"

const float MASS = 0.01;
const float GRAVITY_CONST = 9.80665;
const float K_DRAG = 0.15;
const float RADIUS = 0.25f;

// Initialize the ball with a start state
RigidBall::RigidBall() {
    center = Vector3f(0.0f, 4.0f, 0.0);
    m_vVecState.push_back(Vector3f(0.0f, 4.0f, 0.0f)); //position
    m_vVecState.push_back(Vector3f(0.0f, 0.0f, 0.0)); //velocity

}

// Get the gravity force of the rigid ball
Vector3f RigidBall::getDownwardForce() {
    return MASS * Vector3f(0, -1*GRAVITY_CONST, 0);
}

// Particle system integrator for the ball -> Put all the forces in the vector
std::vector<Vector3f> RigidBall::evalF(std::vector<Vector3f> state) {
    std::vector<Vector3f> f;
    Vector3f allForces;
    Vector3f velocity = getState()[1];
    f.push_back(velocity);
    allForces = getDownwardForce() + extraForces;
    f.push_back(allForces);
    return f;
}

float RigidBall::getRadius() {
    return RADIUS;
}

Vector3f RigidBall::getCenter() {
    return center;
}

void RigidBall::draw(GLProgram &gl) {
    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
    Vector3f pos = getState()[0];
    center = pos;
    gl.updateModelMatrix(Matrix4f::translation(pos));
    drawSphere(RADIUS, 30, 10);
}
