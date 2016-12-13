#include "pendulumsystem.h"

#include <cassert>
#include <math.h>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 50;
const float GRAVITY_CONST = 9.80665;

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
		float f1 = rand_uniform(0.5f, 1.5f);
		float f2 = rand_uniform(-0.5f, 0.5f);
		float f3 = rand_uniform(-0.5f, 0.5f);
		this->m_vVecState.push_back(Vector3f(f1,f2,f3)); //position
		this->m_vVecState.push_back(Vector3f(0,0,0)); //velocity
	    }
	m_vVecState.push_back(Vector3f (1.0f,2.0f,0.0f)); //position of ball
	m_vVecState.push_back(Vector3f (0.0f,0.0f,0.0f)); //velocity of ball
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
	for (unsigned int j=0; j<state.size()-2; j++) {
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
	for (unsigned int j=0; j<state.size()-2; j++) {
		//only need to look at particle positions (even indexes in state vector)
		if (j % 2 == 0 && j!=x_i_index) {
			Vector3f x_j = state[j]; //position of the other particle
			Vector3f r = x_i - x_j;
			if (r.abs() >= 0 && r.abs() <= H_RADIUS) {
				Vector3f delW = calcGradientW(r);
				float rho_j = currentRhos[j];
				float p_j = currentPs[j]; 
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
	for (unsigned int j=0; j<state.size()-2; j++) {
		//only need to look at particle positions (even indexes in state vector)
		if (j % 2 == 0 && j!=x_i_index) {
			Vector3f x_j = state[j]; //position of the other particle
			Vector3f r = x_i - x_j;
			if (r.abs() >= 0 && r.abs() <= H_RADIUS) {
				Vector3f v_j = state[j+1];
				float rho_j = currentRhos[j];				
				float delSquaredW = calcLaplacianW(r);
				Vector3f subForce = MASS*(v_j/rho_j)*delSquaredW;
				force = force + subForce;
			}
		}
	}
	force = MU*force;
	return force;
}

Vector3f PendulumSystem::calculateCombinedForces(std::vector<Vector3f> state, int x_i_index) {
	Vector3f force1(0.0f, 0.0f, 0.0f);	
	Vector3f force2(0.0f, 0.0f, 0.0f);
	Vector3f x_i = state[x_i_index];
	for (unsigned int j=0; j<state.size()-2; j++) {
		//only need to look at particle positions (even indexes in state vector)
		if (j % 2 == 0 && j!=x_i_index) {
			Vector3f x_j = state[j]; //position of the other particle
			Vector3f r = x_i - x_j;
			if (r.abs() >= 0 && r.abs() <= H_RADIUS) {
				Vector3f delW = calcGradientW(r);
				float rho_j = currentRhos[j];
				float p_j = currentPs[j]; 
				Vector3f subForce1 = MASS*(p_j/rho_j)*delW;
				force1 = force1 + subForce1;

				Vector3f v_j = state[j+1];
				float delSquaredW = calcLaplacianW(r);
				Vector3f subForce2 = MASS*(v_j/rho_j)*delSquaredW;
				force2 = force2 + subForce2;
			}
		}
	}
	force1 = -1*force1;
	force2 = MU*force2;

	return force1+force2;
}

std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
	std::vector<float> nextRhos;
	std::vector<float> nextPs;
	//return state list with velocity1, acceleration1, v2, a2, ...
	std::vector<Vector3f> f; //in the form: velocity, acceleration (force/m)
	float ballGravity = -1*GRAVITY_CONST*0.1;
	//initialize f
	for (unsigned int i=1; i<=state.size()-2; i++) {
		//only want gravity forces for particles, not their velocities
		if (i % 2 == 1) {
			f.push_back(state[i]); //add velocities to f
			float thisRho = calculateDensity(state, i-1);
			nextRhos.push_back(thisRho);
			nextPs.push_back(K*(thisRho - RHO_ENV));
		} else {
			//currently no forces
			f.push_back(Vector3f(0.0f, 0.0f, 0.0f)); //add empty vector for forces to f
		}
	}
	currentRhos = nextRhos;
	currentPs = nextPs;

	f.push_back(state[state.size()-1]); //velocity of ball
	f.push_back(Vector3f(0.0f, ballGravity, 0.0f)); //acceleration of ball

	Vector3f f_buoyancy = Vector3f(0.0f, (-1*ALPHA*S) + (BETA*(TEMP - TEMP_ENV)), 0.0f);

	//calculate fluid forces: particle density, particle pressure, force of pressure, force of viscosity
	for (unsigned int i=0; i<state.size()-2; i++) {
		//only need to add this to the forces (odd indexes in f)
		if (i % 2 == 1) {
			//get variables
			int x_i_index = i-1; //position of this particle
			//Vector3f f_pressure_i = calculatePressureForce(state, x_i_index);
			//Vector3f f_viscosity_i = calculateViscosityForce(state, x_i_index);
			Vector3f forces = calculateCombinedForces(state, x_i_index);
			//f[i] = (f_pressure_i + f_viscosity_i + f_buoyancy)/MASS;
			f[i] = (forces + f_buoyancy)/MASS;
		}
	}


	 float ballRadius = 0.25f;
	 Vector3f ballPosition = state[state.size()-2];
	 //list of smoke particles touching the ball
	 for (unsigned int i=0; i<state.size()-2; i++) {
	 	//only need to add this to the forces (odd indexes in f)
	 	if (i % 2 == 1) {
	 		//check if ball intersects with particle:
	 		Vector3f particlePosition = state[i-1];
	 		float distanceBetweenBallAndParticle = (ballPosition-particlePosition).abs();
	 		if (distanceBetweenBallAndParticle <= (ballRadius+0.01)) {
	 			//particle is inside ball
	 			Vector3f currentForce = f[i];
	 			Vector3f newForce = currentForce + Vector3f(0.0f, ballGravity, 0.0f)/MASS;
	 			f[i] = newForce;
	 		}
	 	}
	 }

	return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
	const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
	gl.updateMaterial(PENDULUM_COLOR);

	for (unsigned int i=0; i<m_vVecState.size()/2; i++) {
		Vector3f position = m_vVecState[i*2];
		if (i==((m_vVecState.size()/2)-1)) {
			gl.updateModelMatrix(Matrix4f::translation(position));
			drawSphere(0.25f, 20, 20);
		} else {
			gl.updateModelMatrix(Matrix4f::translation(Vector3f(position)));
			printf("S ");
			position.print();
			gl.updateMaterial(Vector3f(1.0f,1.0f,1.0f),Vector3f(-1,-1,-1), Vector3f(0,0,0),1.0f,0.5f);
			drawSphere(0.05f, 8, 8);
			//drawQuad(0.025);
			int counter = 0;
			while (counter < 35) {

				float f1 = rand_uniform(-0.2f, 0.2f);
				float f2 = rand_uniform(-0.2f, 0.2f);
				float f3 = rand_uniform(-0.2f, 0.2f);

				gl.updateModelMatrix(Matrix4f::translation(Vector3f(position) + Vector3f(f1, f2, f3)));
				gl.updateMaterial(Vector3f(1.0f,1.0f,1.0f),Vector3f(-1,-1,-1), Vector3f(0,0,0),1.0f,0.5f);
				drawSphere(0.05f, 8, 8);
				//drawQuad(0.025);
				counter += 1;
			}



			//drawQuad(0.05f);
		}
	}
    //printf("%s\n", "One Iteration");
}
