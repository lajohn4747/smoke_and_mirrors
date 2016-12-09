#include "pendulumsystem.h"

#include <cassert>
#include <math.h>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 30;
const float GRAVITY_CONST = -9.80665;

const float MASS = 0.1f; //mass of a single smoke particle
const float H_RADIUS = 0.1f; //radius of volume where we care about surrounding particles for fluid flow
const float RHO_ENV = 1.0f; //environmental pressure
const float TEMP_ENV = 1.0f; //environmental/ambient temperature
const float K = 1.0f; //gas constant that depends on temperature?
const float MU = 1.0f; //viscosity constant (very low for smoke)
const float ALPHA = 1.0; //bouancy constant 1
const float BETA = 1.0; //bouancy constant 2
const float PI = 3.14159265358979323846f;


PendulumSystem::PendulumSystem()
{
	for (int i = 1; i < NUM_PARTICLES; i++) {
		// for this system, we care about the position and the velocity
		float f1 = rand_uniform(-0.3f, 0.3f);
		float f2 = rand_uniform(-0.3f, 0.3f);
		float f3 = rand_uniform(-0.3f, 0.3f);
		this->m_vVecState.push_back(Vector3f(f1,f2,f3)); //position
		this->m_vVecState.push_back(Vector3f(0,0,0)); //velocity
	    }
}


float PendulumSystem::calcW(Vector3f r)
{
	//TODO use spiky kernel instead?
	float coefficient = 315.0f/(64*PI*pow(H_RADIUS, 9));
	float doingMath = pow((pow(H_RADIUS,2) - pow(r.abs(), 2)), 3);
	return coefficient*doingMath;
}

Vector3f PendulumSystem::calcGradientW(Vector3f r)
{
	float coefficient = 945.0f/(32*PI*pow(H_RADIUS, 9));
	float middleTerm = pow((pow(H_RADIUS,2) - pow(r.abs(), 2)), 2);
	return Vector3f(coefficient*middleTerm*2*r.x(), coefficient*middleTerm*2*r.y(), coefficient*middleTerm*2*r.z());
}

float PendulumSystem::calcLaplacianW(Vector3f r)
{
	//TODO check viscosity vs. poly 6 kernel
	float coefficient = 45.0f/(PI*pow(H_RADIUS, 6));
	float doingMath = H_RADIUS - r.abs();
	return coefficient*doingMath;

}

float PendulumSystem::calculateDensity(std::vector<Vector3f> state, Vector3f x_i)
{
	float rho_i = 0.0f;
	for (int i=0; i<state.size(); i++) {
		//only need to look at particle positions (even indexes in state vector)
		if (i % 2 == 0) {
			Vector3f x_j = state[i]; //position of the other particle
			Vector3f r = x_i - x_j;
			if (r.abs() >= 0 && r.abs() <= H_RADIUS) {
				//calculate density (rho)
				float W = calcW(r);
				rho_i = rho_i + (MASS*W);
			}
		}
	}
	return rho_i;
}


Vector3f PendulumSystem::calculatePressureForce(std::vector<Vector3f> state, Vector3f x_i, float p, float rho)
{
	Vector3f force(0.0f, 0.0f, 0.0f);
	for (int i=0; i<state.size(); i++) {
		//only need to look at particle positions (even indexes in state vector)
		if (i % 2 == 0) {
			Vector3f x_j = state[i]; //position of the other particle
			Vector3f r = x_i - x_j;
			if (r.abs() >= 0 && r.abs() <= H_RADIUS) {
				Vector3f delW = calcGradientW(r);
				Vector3f subForce = MASS*(p/rho)*delW;
				force = force + subForce;
			}
		}
	}
	force = -1*force;
	return force;
}


Vector3f PendulumSystem::calculateViscosityForce(std::vector<Vector3f> state, Vector3f x_i, Vector3f v_i, float rho)
{
	Vector3f force(0.0f, 0.0f, 0.0f);
	for (int i=0; i<state.size(); i++) {
		//only need to look at particle positions (even indexes in state vector)
		if (i % 2 == 0) {
			Vector3f x_j = state[i]; //position of the other particle
			Vector3f r = x_i - x_j;
			if (r.abs() >= 0 && r.abs() <= H_RADIUS) {
				float delSquaredW = calcLaplacianW(r);
				Vector3f subForce = MASS*(v_i/rho)*delSquaredW;
				force = force + subForce;
			}
		}
	}
	force = MU*force;
	return force;
}

std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
	//return state list with velocity1, acceleration1, v2, a2, ...
	std::vector<Vector3f> f; //in the form: velocity, acceleration (force/m)

	//initialize f
	for (unsigned int i=1; i<=state.size(); i++) {
		//only want gravity forces for particles, not their velocities
		if (i % 2 == 1) {
			f.push_back(state[i]); //add velocities to f
		} else {
			//currently no forces
			f.push_back(Vector3f(0.0f, 0.0f, 0.0f)); //add empty vector for forces to f
		}
	}

	//calculate fluid forces: particle density, particle pressure, force of pressure, force of viscosity
	for (int i=0; i<state.size(); i++) {
		//only need to add this to the forces (odd indexes in f)
		if (i % 2 == 1) {
			//get variables
			Vector3f x_i = state[i-1]; //position of this particle
			Vector3f v_i = state[i]; //velocity of this particle
			float rho_i = calculateDensity(state, x_i); //density of this particle
			float p = K*(rho_i - RHO_ENV); //pressure of the particle
			
			Vector3f f_pressure_i = calculatePressureForce(state, x_i, p, rho_i);
			Vector3f f_viscosity_i = calculateViscosityForce(state, x_i, v_i, rho_i);

			f[i] = (f_pressure_i + f_viscosity_i)/MASS;
		}
	}

	//TODO calculate force of bouyancy, external forces (gravity of the rigid ball)

	return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
	const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
	gl.updateMaterial(PENDULUM_COLOR);

	for (unsigned int i=0; i<m_vVecState.size()/2; i++) {
		Vector3f postition = m_vVecState[i*2];
		gl.updateModelMatrix(Matrix4f::translation(Vector3f(postition)));
		drawSphere(0.075f, 10, 10);
	}
}
