#include "timestepper.h"
#include <math.h>
#include <cstdio>


void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	//TODO: See handout 3.1 
	//get X
	std::vector<Vector3f> currentState = particleSystem->getState();
	//get f(X,t)
	std::vector<Vector3f> derivativeOfCurrentState = particleSystem->evalF(currentState);
	//get X + h*f(X,t) for each vector
	std::vector<Vector3f> newState;
	for (unsigned int i=0; i<derivativeOfCurrentState.size(); i++) {
		Vector3f newSubStep = stepSize*derivativeOfCurrentState[i];
		Vector3f newStep = currentState[i] + newSubStep;
		newState.push_back(newStep);
	}
	particleSystem->setState(newState);
	
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	//TODO: See handout 3.1 
	//get X
	std::vector<Vector3f> currentState = particleSystem->getState();
	//get f0 = f(X,t)
	std::vector<Vector3f> f0 = particleSystem->evalF(currentState);
	//get newX = X + h*f(X,t) for each vector
	std::vector<Vector3f> newX;
	for (unsigned int i=0; i<f0.size(); i++) {
		Vector3f newSubStep = stepSize*f0[i];
		Vector3f newStep = currentState[i] + newSubStep;
		newX.push_back(newStep);
	}
	//get f1=f(newX, t+h)
	std::vector<Vector3f> f1 = particleSystem->evalF(newX);
	//get final output X(t,h) = X + h/2*(f0+f1)
	std::vector<Vector3f> finalState;
	for (unsigned int j=0; j<f0.size(); j++) {
		Vector3f newSubStep2 = (stepSize/2.0)*(f0[j] + f1[j]);
		Vector3f newStep2 = currentState[j] + newSubStep2;
		finalState.push_back(newStep2);
	}
	particleSystem->setState(finalState);
}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	//TODO: See handout 4.4
	//using constants from slide 19, Lecture 8
	//get X
	std::vector<Vector3f> X = particleSystem->getState();
	//get k1 = f(X,t)
	std::vector<Vector3f> k1 = particleSystem->evalF(X);

	//get k2 = evalF(X + h/2*k1)
	std::vector<Vector3f> k2InnerArgument;
	for (unsigned int i=0; i<k1.size(); i++) {
		Vector3f newSubStep = (stepSize/2.0)*k1[i];
		Vector3f newStep = X[i] + newSubStep;
		k2InnerArgument.push_back(newStep);
	}
	//get k2
	std::vector<Vector3f> k2 = particleSystem->evalF(k2InnerArgument);

	//get k3 = evalF(X + h/2*k2)
	std::vector<Vector3f> k3InnerArgument;
	for (unsigned int i=0; i<k2.size(); i++) {
		Vector3f newSubStep = (stepSize/2.0)*k2[i];
		Vector3f newStep = X[i] + newSubStep;
		k3InnerArgument.push_back(newStep);
	}
	//get k3
	std::vector<Vector3f> k3 = particleSystem->evalF(k3InnerArgument);

	//get k4 = evalF(X + h*k3)
	std::vector<Vector3f> k4InnerArgument;
	for (unsigned int i=0; i<k3.size(); i++) {
		Vector3f newSubStep = stepSize*k3[i];
		Vector3f newStep = X[i] + newSubStep;
		k4InnerArgument.push_back(newStep);
	}
	//get k4
	std::vector<Vector3f> k4 = particleSystem->evalF(k4InnerArgument);

	//get final newX new state
	//newX = X + h/6*(k1 + 2k2 + 2k3 + k4)
	std::vector<Vector3f> finalState;
	for (unsigned int j=0; j<X.size(); j++) {
		Vector3f addKs = k1[j] + 2*k2[j] + 2*k3[j] + k4[j];
		Vector3f mathStep2 = X[j] + ((stepSize/6.0)*addKs);
		finalState.push_back(mathStep2);
	}
	particleSystem->setState(finalState);
}

void RKF::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	//implemented using these two papers, in the README file
	float tolerance_epsilon = 0.001;
	float h = stepSize;

	//get X
	std::vector<Vector3f> X = particleSystem->getState();
	//get k1 = h*evalF(X)
	std::vector<Vector3f> k1 = particleSystem->evalF(X);
	for (unsigned int i=0; i<k1.size(); i++) {
		Vector3f k1Old = k1[i];
		k1[i] = h*k1Old;
	}

	//get k2 = h*evalF(X + k1/4)
	std::vector<Vector3f> k2InnerArgument;
	for (unsigned int i=0; i<k1.size(); i++) {
		Vector3f newStep = X[i] + (k1[i]/4.0);
		k2InnerArgument.push_back(newStep);
	}
	//get k2
	std::vector<Vector3f> k2 = particleSystem->evalF(k2InnerArgument);
	for (unsigned int i=0; i<k2.size(); i++) {
		Vector3f k2Old = k2[i];
		k2[i] = h*k2Old;
	}

	//get k3 = h*evalF(X + 3*k1/32 + 9*k2/32)
	std::vector<Vector3f> k3InnerArgument;
	for (unsigned int i=0; i<k2.size(); i++) {
		Vector3f newStep = X[i] + (3.0*k1[i]/32.0) + (9.0*k2[i]/32.0);
		k3InnerArgument.push_back(newStep);
	}
	//get k3
	std::vector<Vector3f> k3 = particleSystem->evalF(k3InnerArgument);
	for (unsigned int i=0; i<k3.size(); i++) {
		Vector3f k3Old = k3[i];
		k3[i] = h*k3Old;
	}

	//get k4 = h*evalF(X + 1932*k1/2197 - 7200*k2/2197 + 7296*k3/2197)
	std::vector<Vector3f> k4InnerArgument;
	for (unsigned int i=0; i<k3.size(); i++) {
		Vector3f newStep = X[i] + (1932.0*k1[i]/2197.0) - (7200.0*k2[i]/2197.0) + (7296.0*k3[i]/2197.0);
		k4InnerArgument.push_back(newStep);
	}
	//get k4
	std::vector<Vector3f> k4 = particleSystem->evalF(k4InnerArgument);
	for (unsigned int i=0; i<k4.size(); i++) {
		Vector3f k4Old = k4[i];
		k4[i] = h*k4Old;
	}

	//get k5 = h*evalF(X + 439*k1/216 - 8*k2 + 3680*k3/513 - 845*k4/4104)
	std::vector<Vector3f> k5InnerArgument;
	for (unsigned int i=0; i<k4.size(); i++) {
		Vector3f newStep = X[i] + (439.0*k1[i]/216.0) - (8.0*k2[i]) + (3680.0*k3[i]/513.0) - (845.0*k4[i]/4104.0);
		k5InnerArgument.push_back(newStep);
	}
	//get k5
	std::vector<Vector3f> k5 = particleSystem->evalF(k5InnerArgument);
	for (unsigned int i=0; i<k5.size(); i++) {
		Vector3f k5Old = k5[i];
		k5[i] = h*k5Old;
	}

	//get k6 = h*evalF(X - 8*k1/27 + 2*k2 - 3544*k3/2565 + 1859*k4/4104 - 11*k5/40)
	std::vector<Vector3f> k6InnerArgument;
	for (unsigned int i=0; i<k5.size(); i++) {
		Vector3f newStep = X[i] - (8.0*k1[i]/27.0) + (2.0*k2[i]) - (3544.0*k3[i]/2565.0) + (1859.0*k4[i]/4104.0) - (11.0*k5[i]/40.0);
		k6InnerArgument.push_back(newStep);
	}
	//get k6
	std::vector<Vector3f> k6 = particleSystem->evalF(k6InnerArgument);
	for (unsigned int i=0; i<k6.size(); i++) {
		Vector3f k6Old = k6[i];
		k6[i] = h*k6Old;
	}
	
	//check next state1
	std::vector<Vector3f> potentialNextX1;
	for (unsigned int i=0; i<X.size(); i++) {
		Vector3f mathing = X[i] + (25.0*k1[i]/216.0) + (1408.0*k3[i]/2565.0) + (2197.0*k4[i]/4104.0) - (k5[i]/5.0);
		potentialNextX1.push_back(mathing);
	}

	//check next state2
	std::vector<Vector3f> potentialNextX2;
	for (unsigned int i=0; i<X.size(); i++) {
		Vector3f mathing = X[i] + (16.0*k1[i]/135.0) + (6656.0*k3[i]/12825.0) + (28561.0*k4[i]/56430.0) - (9.0*k5[i]/50.0) + (2.0*k6[i]/55.0);;
		potentialNextX2.push_back(mathing);
	}
	
	//find error for each component
	std::vector<float> errorVec;
	for (unsigned int j=0; j<X.size(); j++) {
		float errorDifference = Vector3f(potentialNextX1[j] - potentialNextX2[j]).abs();
		float error = (1.0/h)*errorDifference;
		errorVec.push_back(error);
	}
	//now get the average error to compare to the tolerence
	float averageError = 0;
	for (unsigned int j=0; j<errorVec.size(); j++) {
		averageError += errorVec[j];
	}
	averageError = averageError/errorVec.size();

	if (averageError <= tolerance_epsilon) {
		particleSystem->setState(potentialNextX1);
	} else {
		float delta = 0.84*pow ((tolerance_epsilon/averageError), 0.25);
		RKF::takeStep(particleSystem, delta*h);
	}

}

