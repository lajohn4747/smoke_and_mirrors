#include "pendulumsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 30;
const float GRAVITY_CONST = -9.80665;
const bool DEBUG = false;

const float MASS = 2.0;
const float K_DRAG = -0.01;
const float REST_LENGTH = 0.01;
const float K_SPRING = 2.0;


PendulumSystem::PendulumSystem()
{

	// TODO 4.2 Add particles for simple pendulum
	// TODO 4.3 Extend to multiple particles
	//state has structure: position1, velocity1, position2, velocity2, ...
	//point 1
	//m_vVecState.push_back(Vector3f (-1.0f,2.04f,0.0f)); //position
	//m_vVecState.push_back(Vector3f (0.0f,0.0f,0.0f)); //velocity
	//point 2
	//m_vVecState.push_back(Vector3f (-1.0f,2.02f,0.0f)); //position
	//m_vVecState.push_back(Vector3f (0.0f,0.0f,0.0f)); //velocity
	//point 3
	//m_vVecState.push_back(Vector3f (-1.0f,2.01f,0.0f)); //position
	//m_vVecState.push_back(Vector3f (0.0f,0.0f,0.0f)); //velocity
	//point 4
	//m_vVecState.push_back(Vector3f (-1.0f,2.00f,0.0f)); //position
	//m_vVecState.push_back(Vector3f (0.0f,0.0f,0.0f)); //velocity


for (int i = 1; i < NUM_PARTICLES; i++) {
        // for this system, we care about the position and the velocity
        float f1 = rand_uniform(-0.3f, 0.3f);
        float f2 = rand_uniform(-0.3f, 0.3f);
	float f3 = rand_uniform(-0.3f, 0.3f);
        this->m_vVecState.push_back(Vector3f(f1,f2,f3));
        this->m_vVecState.push_back(Vector3f(0,0,0));
    }

	//springs storage
	//length should be m_vVecState.size()/2 -1
	//syntax of a spring Vector4f: index of particle 1, index of particle 2, rest lenght, stiffness
	//spring between point 1 and point 2	
	m_springs.push_back(Vector4f(0.0f, 1.0f, REST_LENGTH, K_SPRING));
	//spring between point 2 and point 3
	m_springs.push_back(Vector4f(1.0f, 2.0f, REST_LENGTH, K_SPRING));
	//spring between point 3 and point 4
	m_springs.push_back(Vector4f(2.0f, 3.0f, REST_LENGTH, K_SPRING));

	// To add a bit of randomness, use e.g.
	// float f = rand_uniform(-0.5f, 0.5f);
	// in your initial conditions.
}

std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
	//return state list with velocity1, acceleration1, v2, a2, ...
	std::vector<Vector3f> f;
	// TODO 4.1: implement evalF
	//  - gravity
	//  - viscous drag
	//  - springs

	//gravity: m*g in the negative y direction
	//also add velocities here
	//std::vector<Vector3f> gravity_forces;
	for (unsigned int i=1; i<=state.size(); i++) {
		//only want gravity forces for particles, not their velocities
		if (i % 2 == 1) {
			f.push_back(state[i]);
		} else {
			//don't bother multiplying by mass, since we divide this out anyway
			f.push_back(Vector3f(0.0f, -1*GRAVITY_CONST, 0.0f));
		}
	}

	//viscous drag: -k*velocity
	//std::vector<Vector3f> viscous_drag_forces;
	for (unsigned int i=0; i<state.size(); i++) {
		//only need to add this to the gravity forces (odd indexes in f)
		if (i % 2 == 1) {
			Vector3f currentForce = f[i];
			Vector3f dragForce = -1*K_DRAG*state[i];
			Vector3f newForce = currentForce + (dragForce/MASS);
			f[i] = newForce;
		}
	}



	return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
	const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
	gl.updateMaterial(PENDULUM_COLOR);

	// TODO 4.2, 4.3
	for (unsigned int i=0; i<m_vVecState.size()/2; i++) {
		Vector3f postition = m_vVecState[i*2];
		gl.updateModelMatrix(Matrix4f::translation(Vector3f(postition)));
		drawSphere(0.075f, 10, 10);
	}

	//draw springs in debug mode
	if (DEBUG) {
		gl.disableLighting();
		gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
		VertexRecorder rec;
		for (unsigned int i=0; i<m_springs.size(); i++) {
			Vector4f spring = m_springs[i];
			int particle_1_position_index = (int) spring[0]*2;
			int particle_2_position_index = (int) spring[1]*2;
			Vector3f particle_1_position = m_vVecState[particle_1_position_index];
			Vector3f particle_2_position = m_vVecState[particle_2_position_index];

			rec.record(particle_1_position, PENDULUM_COLOR);
			rec.record(particle_2_position, PENDULUM_COLOR);
		}
		glLineWidth(3.0f);
		rec.draw(GL_LINES);
		gl.enableLighting();
	}

	// example code. Replace with your own drawing  code
	//gl.updateModelMatrix(Matrix4f::translation(Vector3f(-0.5, 1.0, 0)));
	//drawSphere(0.075f, 10, 10);
}
