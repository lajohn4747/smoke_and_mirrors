#include "pendulumsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
using namespace std;
PendulumSystem::PendulumSystem()
{

    // TODO 4.2 Add particles for simple pendulum
    m_vVecState.push_back(Vector3f(0,0,0));
    m_vVecState.push_back(Vector3f(0,0,0));
    // TODO 4.3 Extend to multiple particles
    for (int i = 1; i < NUM_PARTICLES; i++) {
        // for this system, we care about the position and the velocity
        float f1 = rand_uniform(-0.3f, 0.3f);
        float f2 = rand_uniform(-0.3f, 0.3f);
        this->m_vVecState.push_back(Vector3f(f1,f2,0));
        this->m_vVecState.push_back(Vector3f(0,0,0));
    }

    // To add a bit of randomness, use e.g.
    //float f = rand_uniform(-0.5f, 0.5f);
    // in your initial conditions.
}

Vector3f PendulumSystem::computeGravity(float mass){
    return mass * Vector3f(0,-9.8, 0);
}

Vector3f PendulumSystem::computeDrag(float k, Vector3f velocity){
    return -k * velocity;
}

Vector3f PendulumSystem::computeSpring(float k, float r, int i, int j, vector<Vector3f> pos) {
    if (i < 0 || i >= NUM_PARTICLES || j < 0 || j >= NUM_PARTICLES){
        return Vector3f(0,0,0);
    }
    Vector3f difference = pos[i] - pos[j];
    return -k * (difference.abs() - r) * difference.normalized();
}

vector<Vector3f> PendulumSystem::getPositions(vector<Vector3f> state){
    vector<Vector3f> pos;
    for(unsigned i = 0;i<state.size(); i+=2){
        pos.push_back(state[i]);
    }
    return pos;
}

vector<Vector3f> PendulumSystem::getVelocities(vector<Vector3f> state){
    vector<Vector3f> vel;
    for(unsigned i = 1; i<state.size(); i+=2){
        vel.push_back(state[i]);
    }
    return vel;
}


std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    float mass = .25;
    float r = 0.3;
    float drag = 0.1;
    float springK = 8.5;
    vector<Vector3f> f;
    vector<Vector3f> positions = getPositions(state);
    vector<Vector3f> velocities = getVelocities(state);
    // Pushing back the first velocity and first force which is the tethered particle
    f.push_back(Vector3f(0,0,0));
    f.push_back(Vector3f(0,0,0));
    for(unsigned i = 1; i < NUM_PARTICLES; ++i) {
        f.push_back(velocities[i]);
        //Vector3f springForce = computeSpring(springK,r,i,i-1,positions) + computeSpring(springK,r,i,i+1,positions)
          //                    + computeSpring(springK,2*r,i,i+2,positions) + computeSpring(springK,2*r,i,i-2,positions);
        Vector3f springForce =  computeSpring(springK,r,i,i-1,positions) + computeSpring(springK,r,i,i+1,positions);
        /*springForce2.print();
        printf("%d %d\n", i, i+1);
        printf("\n");*/
        Vector3f forces = (computeGravity(mass) + computeDrag(drag,velocities[i]) + springForce)/mass;
        f.push_back(forces);


        //  - gravity
        //  - viscous drag
        //  - springs
    }
    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // TODO 4.2, 4.3
    vector<Vector3f> positions = getPositions(getState());
    for(unsigned i = 0; i < NUM_PARTICLES; ++i){
        Vector3f pos = positions[i];
        gl.updateModelMatrix(Matrix4f::translation(pos));
        drawSphere(0.075f, 10, 10);
    }
    // example code. Replace with your own drawing  code
    //gl.updateModelMatrix(Matrix4f::translation(Vector3f(-0.5, 1.0, 0)));
    //drawSphere(0.075f, 10, 10);
}
