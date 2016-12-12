#include "pendulumsystem.h"

#include <cassert>
#include <math.h>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 30;
const float GRAVITY_CONST = -9.80665;

const float MASS = 0.1f; //mass of a single smoke particle
const float H_RADIUS = 0.01f; //radius of volume where we care about surrounding particles for fluid flow
const float RHO_ENV = 1.0f; //environmental pressure
const float TEMP_ENV = 300.0f; //environmental/ambient temperature
const float K = 100.0f; //gas constant that depends on temperature?
const float MU = 0.000018f; //viscosity constant (very low for smoke)
const float ALPHA = 1.0; //bouancy constant 1
const float BETA = 0.001; //bouancy constant 2
const float S = 0.05f; //smoke concentration
const float TEMP = 500.0f; //temperature of smoke
const float PI = 3.14159265358979323846f;


PendulumSystem::PendulumSystem() {
	for (int i = 1; i < NUM_PARTICLES; i++) {
		// for this system, we care about the position and the velocity
		float f1 = rand_uniform(-0.3f, 0.3f);
		float f2 = rand_uniform(-0.3f, 0.3f);
		float f3 = rand_uniform(-0.3f, 0.3f);
		this->m_vVecState.push_back(Vector3f(f1,f2,f3)); //position
		this->m_vVecState.push_back(Vector3f(0,0,0)); //velocity
	    }
}


float PendulumSystem::calcW(Vector3f r) {
	//using poly6 kernel from Muller paper
	float coefficient = 315.0f/(64*PI*pow(H_RADIUS, 9));
	float doingMath = pow((pow(H_RADIUS,2) - pow(r.abs(), 2)), 3);
	return coefficient*doingMath;
}

Vector3f PendulumSystem::calcGradientW(Vector3f r) {
	//using spiky kernel from Muller paper
	float coefficient = 15.0f/(PI*pow(H_RADIUS, 6));
	float middleTerm = -3*pow(H_RADIUS-r.abs(), 2);
	return Vector3f(coefficient*middleTerm*r.x(), coefficient*middleTerm*r.y(), coefficient*middleTerm*r.z());

	//old poly6 kernel code
	/*float coefficient = 945.0f/(32*PI*pow(H_RADIUS, 9)); //WRONG - need to multiply by r
	float middleTerm = pow((pow(H_RADIUS,2) - pow(r.abs(), 2)), 2);
	return Vector3f(coefficient*middleTerm*2*r.x(), coefficient*middleTerm*2*r.y(), coefficient*middleTerm*2*r.z());
	*/
}

float PendulumSystem::calcLaplacianW(Vector3f r) {
	//using viscosity kernel from Muller paper
	float coefficient = 45.0f/(PI*pow(H_RADIUS, 6));
	float doingMath = H_RADIUS - r.abs();
	return coefficient*doingMath;
}

float PendulumSystem::calculateDensity(std::vector<Vector3f> state, int x_i_index) {
	float rho_i = 0.0f;
	Vector3f x_i = state[x_i_index];
	for (int j=0; j<state.size(); j++) {
		//only need to look at particle positions (even indexes in state vector)
		if (j % 2 == 0 && j!=x_i_index) {
			Vector3f x_j = state[j]; //position of the other particle
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


Vector3f PendulumSystem::calculatePressureForce(std::vector<Vector3f> state, int x_i_index) {
	Vector3f force(0.0f, 0.0f, 0.0f);
	Vector3f x_i = state[x_i_index];
	for (int j=0; j<state.size(); j++) {
		//only need to look at particle positions (even indexes in state vector)
		if (j % 2 == 0 && j!=x_i_index) {
			Vector3f x_j = state[j]; //position of the other particle
			Vector3f r = x_i - x_j;
			if (r.abs() >= 0 && r.abs() <= H_RADIUS) {
				Vector3f delW = calcGradientW(r);
				float rho_j = calculateDensity(state, j);
				float p_j = K*(rho_j - RHO_ENV); 
				Vector3f subForce = MASS*(p_j/rho_j)*delW;
				force = force + subForce;
			}
		}
	}
	force = -1*force;
	return force;
}


Vector3f PendulumSystem::calculateViscosityForce(std::vector<Vector3f> state, int x_i_index) {
	Vector3f force(0.0f, 0.0f, 0.0f);
	Vector3f x_i = state[x_i_index];
	for (int j=0; j<state.size(); j++) {
		//only need to look at particle positions (even indexes in state vector)
		if (j % 2 == 0 && j!=x_i_index) {
			Vector3f x_j = state[j]; //position of the other particle
			Vector3f r = x_i - x_j;
			if (r.abs() >= 0 && r.abs() <= H_RADIUS) {
				Vector3f v_j = state[j+1];
				float rho_j = calculateDensity(state, j);				
				float delSquaredW = calcLaplacianW(r);
				Vector3f subForce = MASS*(v_j/rho_j)*delSquaredW;
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

	Vector3f f_buoyancy = Vector3f(0.0f, (-1*ALPHA*S) + (BETA*(TEMP - TEMP_ENV)), 0.0f);

	//calculate fluid forces: particle density, particle pressure, force of pressure, force of viscosity
	for (int i=0; i<state.size(); i++) {
		//only need to add this to the forces (odd indexes in f)
		if (i % 2 == 1) {
			//get variables
			int x_i_index = i-1; //position of this particle
			Vector3f f_pressure_i = calculatePressureForce(state, x_i_index);
			Vector3f f_viscosity_i = calculateViscosityForce(state, x_i_index);
			f[i] = (f_pressure_i + f_viscosity_i + f_buoyancy)/MASS;
		}
	}

	//TODO calculate external forces (gravity of the rigid ball)

	return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
	const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
	gl.updateMaterial(PENDULUM_COLOR);

	for (unsigned int i=0; i<m_vVecState.size()/2; i++) {
		Vector3f position = m_vVecState[i*2];
        printf("S ");
        position.print();
		gl.updateModelMatrix(Matrix4f::translation(Vector3f(position)));
		gl.updateMaterial(Vector3f(1.0f,1.0f,1.0f),Vector3f(-1,-1,-1), Vector3f(0,0,0),1.0f,0.5f);
		drawSphere(0.05f, 8, 8);
	}
    //printf("%s\n", "One Iteration");
}
