#include "simplesystem.h"

#include "camera.h"
#include "vertexrecorder.h"


SimpleSystem::SimpleSystem()
{
	// TODO 3.2 initialize the simple system
	//m_mvVecState is a vector<Vector3f>, where the first vector is the position
	//add position
	m_vVecState.push_back(Vector3f(1.0f, 0.0f, 0.0f));
	//add velocity - or not because that breaks things
	//m_vVecState.push_back(Vector3f(0.0f, 0.0f, 0.0f));
}

std::vector<Vector3f> SimpleSystem::evalF(std::vector<Vector3f> state)
{
	std::vector<Vector3f> f;

	// TODO 3.2: implement evalF
	// for a given state, evaluate f(X,t)
	//turn X (aka state) into f(X,t)
	//so turn (x,y,z) into (-y,x,0)
	for (unsigned int i=0; i<state.size(); i++) {
		Vector3f derivative = Vector3f(-1*state[i][1], state[i][0], 0.0);
		f.push_back(derivative);
	}

	return f;
}

// render the system (ie draw the particles)
void SimpleSystem::draw(GLProgram& gl)
{

	// TODO 3.2: draw the particle. 
	//           we provide code that draws a static sphere.
	//           you should replace it with your own
	//           drawing code.
	//           In this assignment, you must manage two
	//           kinds of uniforms before you draw
	//            1. Update material uniforms (color)
	//            2. Update transform uniforms
	//           GLProgram is a helper object that has
	//           methods to set the uniform state.

	const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
	gl.updateMaterial(PARTICLE_COLOR);
	//Vector3f pos(1, 0, 0); //YOUR PARTICLE POSITION
	std::vector<Vector3f> state = getState();
	Vector3f pos(state[state.size()-1]); //or state[0], they both seem to do the same thing here because we only have one particle
	gl.updateModelMatrix(Matrix4f::translation(pos));
	drawSphere(0.075f, 10, 10);
}
