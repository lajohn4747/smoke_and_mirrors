#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>
#include <math.h> //acos
#include <algorithm> //max and min

using namespace std;


 // your system should at least contain 8x8 particles.
const int W = 8;
const int H = 8;
const float MASS = 0.035;
const float GRAVITY_CONST = 9.80665;
const float K_DRAG = 0.15;
const float STRUCTURAL_REST = 0.3;
const float STRUCTURAL_K = 20.0;
const float SHEAR_REST = 0.424264;
const float SHEAR_K = 30.0;
const float FLEXION_REST = 0.6;
const float FLEXION_K = 30.0;

ClothSystem::ClothSystem()
{
	// TODO 5. Initialize m_vVecState with cloth particles. 
	// You can again use rand_uniform(lo, hi) to make things a bit more interesting

	//state has structure: position1, velocity1, position2, velocity2, ...
	float x_value = -1.05f;
	float z_value = -1.05f;
	for (unsigned int i=0; i<W; i++) {
		float thisX = x_value+(i*0.3f);
		for (unsigned int j=0; j<H; j++) {
			float thisZ = z_value+(j*0.3f);
			m_vVecState.push_back(Vector3f (thisX,0.0f,thisZ)); //position
			m_vVecState.push_back(Vector3f (0.0f,0.0f,0.0f)); //velocity
		}
	}

	m_vVecState.push_back(Vector3f (0.0f,2.0f,0.0f)); //position of ball
	m_vVecState.push_back(Vector3f (0.0f,0.0f,0.0f)); //velocity of ball

	//springs
	//syntax of a spring Vector4f: [index of particle 1 position, index of particle 2 position, rest lenght, stiffness]
	
	//initialize structural springs
	for (unsigned int i=0; i<W; i++) {
		for (unsigned int j=0; j<H; j++) {
			if (i+1 < W){
				m_structuralSprings.push_back(Vector4f (indexOf(i, j), indexOf(i+1, j),STRUCTURAL_REST, STRUCTURAL_K));
			}
			if (j+1 < H) {
				m_structuralSprings.push_back(Vector4f (indexOf(i, j), indexOf(i, j+1),STRUCTURAL_REST, STRUCTURAL_K));
			}
		}
	}

	//initialize shear springs
	for (int i=0; i<W; i++) {
		for (int j=0; j<H; j++) {
			if (i+1 < W && j+1 < H){
				int firstIndex = indexOf(i,j);
				int secondIndex = indexOf(i+1,j+1);
				m_shearSprings.push_back(Vector4f (firstIndex, secondIndex,SHEAR_REST, SHEAR_K));
			}

			if ( (i-1) >= 0 && j+1 < H) {
				int firstIndex = indexOf(i,j);
				int secondIndex = indexOf(i-1,j+1);
				m_shearSprings.push_back(Vector4f (firstIndex, secondIndex,SHEAR_REST, SHEAR_K));
			}
		}
	}

	//initialize flexion strings
	for (unsigned int i=0; i<W; i++) {
		for (unsigned int j=0; j<H; j++) {
			if (i+2 < W){
				m_flexionSprings.push_back(Vector4f (indexOf(i, j), indexOf(i+2, j),FLEXION_REST, FLEXION_K));
			}
			if (j+2 < H) {
				m_flexionSprings.push_back(Vector4f (indexOf(i, j), indexOf(i, j+2),FLEXION_REST, FLEXION_K));
			}
		}
	}

}

//gives the index of the position vector3f if particle (i,j)
int ClothSystem::indexOf(int i, int j) {
	int returnVar = (i*H*2) + (j*2);
	return returnVar;
}

std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    // TODO 5. implement evalF
    // - gravity
    // - viscous drag
    // - structural springs
    // - shear springs
    // - flexion springs
	//gravity: m*g in the negative y direction
	//also add velocities here
	for (unsigned int i=1; i<=state.size(); i++) {
		//only want gravity forces for particles, not their velocities
		if (i==state.size()) {
			f.push_back(Vector3f(0.0f, -1*GRAVITY_CONST*0.1, 0.0f)); //ball forces
		}else if (i % 2 == 1) {
			f.push_back(state[i]);
		} else {
			//don't bother multiplying by mass, since we divide this out anyway
			f.push_back(Vector3f(0.0f, -1*GRAVITY_CONST, 0.0f));
		}
	}

	//viscous drag: -k*velocity
	for (unsigned int i=0; i<state.size()-2; i++) {
		//only need to add this to the gravity forces (odd indexes in f)
		if (i % 2 == 1) {
			Vector3f currentForce = f[i];
			Vector3f dragForce = -1*K_DRAG*state[i];
			Vector3f newForce = currentForce + (dragForce/MASS);
			f[i] = newForce;
		}
	}

	// structural springs
	for (unsigned int i=0; i<m_structuralSprings.size(); i++) {
		Vector4f spring = m_structuralSprings[i];
		int particle_1_position_index = (int) spring[0];
		int particle_2_position_index = (int) spring[1];
		Vector3f particle_1_position = state[particle_1_position_index];
		Vector3f particle_2_position = state[particle_2_position_index];
		Vector3f d = particle_1_position - particle_2_position;
		float k = spring[3];
		float r = spring[2];
		Vector3f springForce = -1*k*(d.abs() - r)*(d/d.abs());
		
		//add spring force to particle 1
		Vector3f forceOnParticle1 = f[particle_1_position_index + 1];
		f[particle_1_position_index + 1] = forceOnParticle1 + (springForce/MASS);

		//add opposite spring force to particle 2
		Vector3f forceOnParticle2 = f[particle_2_position_index + 1];
		f[particle_2_position_index + 1] = forceOnParticle2 + (-1*springForce/MASS);
	}
	//shear springs
	for (unsigned int i=0; i<m_shearSprings.size(); i++) {
		Vector4f spring = m_shearSprings[i];
		int particle_1_position_index = (int) spring[0];
		int particle_2_position_index = (int) spring[1];
		Vector3f particle_1_position = state[particle_1_position_index];
		Vector3f particle_2_position = state[particle_2_position_index];
		Vector3f d = particle_1_position - particle_2_position;
		float k = spring[3];
		float r = spring[2];
		Vector3f springForce = -1*k*(d.abs() - r)*(d/d.abs());
		
		//add spring force to particle 1
		Vector3f forceOnParticle1 = f[particle_1_position_index + 1];
		f[particle_1_position_index + 1] = forceOnParticle1 + (springForce/MASS);

		//add opposite spring force to particle 2
		Vector3f forceOnParticle2 = f[particle_2_position_index + 1];
		f[particle_2_position_index + 1] = forceOnParticle2 + (-1*springForce/MASS);
	}
	//flexion strings
	for (unsigned int i=0; i<m_flexionSprings.size(); i++) {
		Vector4f spring = m_flexionSprings[i];
		int particle_1_position_index = (int) spring[0];
		int particle_2_position_index = (int) spring[1];
		Vector3f particle_1_position = state[particle_1_position_index];
		Vector3f particle_2_position = state[particle_2_position_index];
		Vector3f d = particle_1_position - particle_2_position;
		float k = spring[3];
		float r = spring[2];
		Vector3f springForce = -1*k*(d.abs() - r)*(d/d.abs());
		
		//add spring force to particle 1
		Vector3f forceOnParticle1 = f[particle_1_position_index + 1];
		f[particle_1_position_index + 1] = forceOnParticle1 + (springForce/MASS);

		//add opposite spring force to particle 2
		Vector3f forceOnParticle2 = f[particle_2_position_index + 1];
		f[particle_2_position_index + 1] = forceOnParticle2 + (-1*springForce/MASS);
	}

	float ballRadius = 0.25f;
	Vector3f ballPosition = state[state.size()-2];
	float ballGravity = -0.05*GRAVITY_CONST*500; //last number is ball mass
	//list of cloth particles touching the ball
	std::vector<int> particlesTouchingBall;
	for (int i=0; i<state.size()-2; i++) {
		//only need to add this to the gravity forces (odd indexes in f)
		if (i % 2 == 1) {
			//check if ball intersects with particle:
			Vector3f particlePosition = state[i-1];
			float distanceBetweenBallAndParticle = (ballPosition-particlePosition).abs();
			if (distanceBetweenBallAndParticle <= (ballRadius+0.001)) {
				//particle is inside ball
				//Vector3f currentForce = f[i];
				//Vector3f newForce = currentForce + Vector3f(0.0f, ballGravity, 0.0f);
				//f[i] = newForce;
				particlesTouchingBall.push_back(i-1);
			}
		}
	}


	Vector3f forceOnBallFromCloth(0.0f, 0.0f, 0.0f);
	//find force exerted by cloth on the ball
	for (unsigned int i=0; i<particlesTouchingBall.size(); i++) {
		int particle_index = particlesTouchingBall[i];
			// for each of the <=12 springs attached to a particle, find their force in the +y direction
			// and sum up these forces to get the total force acting back on the ball
			//start with just the structural springs:
			for (unsigned int i=0; i<m_structuralSprings.size(); i++) {
				Vector4f spring = m_structuralSprings[i];
				int particle_1_position_index = (int) spring[0];
				int particle_2_position_index = (int) spring[1];
				if (particle_index == particle_1_position_index || particle_index == particle_2_position_index) {
					Vector3f particle_1_position = state[particle_1_position_index];
					Vector3f particle_2_position = state[particle_2_position_index];
					Vector3f d = particle_1_position - particle_2_position; //distance between the 2 particles
					float k = spring[3]; //k spring constant
					float r = spring[2]; //resting length of spring
					Vector3f springForce = -1*k*(d.abs() - r)*(d/d.abs());

					//get the angle between the line create by the 2 particles and the +y axis
					//math.stackexchange.com/questions/714378
					//float particle_1_y_position = particle_1_position.y();
					//float particle_2_y_position = particle_2_position.y();
					//float max_y_position = std::max(particle_1_y_position, particle_2_y_position);
					//float min_y_position = std::min(particle_1_y_position, particle_2_y_position);
					//float numerator = max_y_position - min_y_position;
					//float demoninator = sqrt(pow(particle_1_position.x()-particle_2_position.x(), 2) + pow(numerator, 2));
					//float innerAngle = acos(numerator/demoninator);

					//now calculate force in +y direction:
					//Vector3f y_force = springForce*cos(innerAngle);
					Vector3f y_force = Vector3f(0.0f, springForce.y(), 0.0f);
					//add spring force to force acting on ball
					if (particle_index == particle_1_position_index) {
						forceOnBallFromCloth = (forceOnBallFromCloth + y_force);
					} else {
						forceOnBallFromCloth = (forceOnBallFromCloth - y_force);
					}
				}
			}
			//shear springs
			for (unsigned int i=0; i<m_shearSprings.size(); i++) {
				Vector4f spring = m_shearSprings[i];
				int particle_1_position_index = (int) spring[0];
				int particle_2_position_index = (int) spring[1];
				if (particle_index == particle_1_position_index || particle_index == particle_2_position_index) {
					Vector3f particle_1_position = state[particle_1_position_index];
					Vector3f particle_2_position = state[particle_2_position_index];
					Vector3f d = particle_1_position - particle_2_position;
					float k = spring[3];
					float r = spring[2];
					Vector3f springForce = -1*k*(d.abs() - r)*(d/d.abs());
					Vector3f y_force = Vector3f(0.0f, springForce.y(), 0.0f);
					//add spring force to force acting on ball
					if (particle_index == particle_1_position_index) {
						forceOnBallFromCloth = (forceOnBallFromCloth + y_force);
					} else {
						forceOnBallFromCloth = (forceOnBallFromCloth - y_force);
					}
				}
			}
			//flexion strings
			for (unsigned int i=0; i<m_flexionSprings.size(); i++) {
				Vector4f spring = m_flexionSprings[i];
				int particle_1_position_index = (int) spring[0];
				int particle_2_position_index = (int) spring[1];
				if (particle_index == particle_1_position_index || particle_index == particle_2_position_index) {
					Vector3f particle_1_position = state[particle_1_position_index];
					Vector3f particle_2_position = state[particle_2_position_index];
					Vector3f d = particle_1_position - particle_2_position;
					float k = spring[3];
					float r = spring[2];
					Vector3f springForce = -1*k*(d.abs() - r)*(d/d.abs());
					Vector3f y_force = Vector3f(0.0f, springForce.y(), 0.0f);
					//add spring force to force acting on ball
					if (particle_index == particle_1_position_index) {
						forceOnBallFromCloth = (forceOnBallFromCloth + y_force);
					} else {
						forceOnBallFromCloth = (forceOnBallFromCloth - y_force);
					}
				}
			}

	}

	//add force applied by ball to cloth
	for (int i=0; i<particlesTouchingBall.size(); i++) {
		int particleIndex = particlesTouchingBall[i];
		Vector3f currentForce = f[particleIndex+1];
		Vector3f newForce = currentForce + Vector3f(0.0f, ballGravity, 0.0f);
		f[particleIndex+1] = newForce;
	}


	//apply forces from cloth on ball
	if (ballPosition.y() < -0.02 && ballPosition.y() > -1.6) {
		//cout << "ball position: " << ballPosition.y() << endl;
		//cout << "force of gravity on ball: " << f[state.size()-1].y() << endl;
	}
	f[state.size()-1] = f[state.size()-1] + forceOnBallFromCloth;
	if (ballPosition.y() < -0.02 && ballPosition.y() > -1.6) {
		//cout << "new force on ball: " << f[state.size()-1].y() << endl;
	}
	
	//finally, make sure we have one fixed point (forces = 0)
	f[indexOf(0,0) + 1] = Vector3f(0.0f,0.0f,0.0f);
	f[indexOf(W-1,0) + 1] = Vector3f(0.0f,0.0f,0.0f);
	f[indexOf(W-1,H-1) + 1] = Vector3f(0.0f,0.0f,0.0f);
	f[indexOf(0,H-1) + 1] = Vector3f(0.0f,0.0f,0.0f);
     
    return f;
}


void ClothSystem::draw(GLProgram& gl)
{
    //TODO 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh

    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);

    // EXAMPLE for how to render cloth particles.
    //  - you should replace this code.
    //float w = 0.2f;
    //Vector3f O(0.4f, 1, 0);

    for (unsigned int i=0; i<m_vVecState.size(); i++) {
	if (i%2 == 0) {
		if (i==m_vVecState.size()-2) {
			gl.updateModelMatrix(Matrix4f::translation(m_vVecState[i]));
    			drawSphere(0.25f, 20, 20);
		}else {
			gl.updateModelMatrix(Matrix4f::translation(m_vVecState[i]));
    			drawSphere(0.04f, 8, 8);
		}
	}
    }
    //gl.updateModelMatrix(Matrix4f::translation(O));
    //drawSphere(0.04f, 8, 8);
    //gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, 0, 0)));
    //drawSphere(0.04f, 8, 8);
    //gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, -w, 0)));
    //drawSphere(0.04f, 8, 8);
    //gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(0, -w, 0)));
    //drawSphere(0.04f, 8, 8);

    // EXAMPLE: This shows you how to render lines to debug the spring system.
    //
    //          You should replace this code.
    //
    //          Since lines don't have a clearly defined normal, we can't use
    //          a regular lighting model.
    //          GLprogram has a "color only" mode, where illumination
    //          is disabled, and you specify color directly as vertex attribute.
    //          Note: enableLighting/disableLighting invalidates uniforms,
    //          so you'll have to update the transformation/material parameters
    //          after a mode change.
    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;
	
	//draw structural Springs
	for (unsigned int i=0; i<m_structuralSprings.size(); i++) {
		Vector4f spring = m_structuralSprings[i];
		int particle_1_position_index = (int) spring[0];
		int particle_2_position_index = (int) spring[1];
		Vector3f particle_1_position = m_vVecState[particle_1_position_index];
		Vector3f particle_2_position = m_vVecState[particle_2_position_index];
		rec.record(particle_1_position, CLOTH_COLOR);
		rec.record(particle_2_position, CLOTH_COLOR);
	}
	//draw shear Springs
	for (unsigned int i=0; i<m_shearSprings.size(); i++) {
		Vector4f spring = m_shearSprings[i];
		int particle_1_position_index = (int) spring[0];
		int particle_2_position_index = (int) spring[1];
		Vector3f particle_1_position = m_vVecState[particle_1_position_index];
		Vector3f particle_2_position = m_vVecState[particle_2_position_index];
		rec.record(particle_1_position, CLOTH_COLOR);
		rec.record(particle_2_position, CLOTH_COLOR);
	}
	//draw flexion Springs
	for (unsigned int i=0; i<m_flexionSprings.size(); i++) {
		Vector4f spring = m_flexionSprings[i];
		int particle_1_position_index = (int) spring[0];
		int particle_2_position_index = (int) spring[1];
		Vector3f particle_1_position = m_vVecState[particle_1_position_index];
		Vector3f particle_2_position = m_vVecState[particle_2_position_index];
		rec.record(particle_1_position, CLOTH_COLOR);
		rec.record(particle_2_position, CLOTH_COLOR);
	}


    //rec.record(O, CLOTH_COLOR);
    //rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
    //rec.record(O, CLOTH_COLOR);
    //rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);

    //rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
    //rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);

    //rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);
    //rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);
    glLineWidth(3.0f);
    rec.draw(GL_LINES);

    gl.enableLighting(); // reset to default lighting model
    // EXAMPLE END
}

