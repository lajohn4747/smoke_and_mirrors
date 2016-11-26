#include "timestepper.h"

#include <cstdio>

using namespace std;
void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
    vector<Vector3f> newValue;
    vector<Vector3f> currentValue = particleSystem->getState();
    vector<Vector3f> f = particleSystem->evalF(currentValue);

    for (unsigned i = 0; i< currentValue.size(); ++i){
        Vector3f updateValue = currentValue[i] + stepSize*f[i];
        newValue.push_back(updateValue);
    }

    particleSystem->setState(newValue);

}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
    vector<Vector3f> newValue;
    vector<Vector3f> currentValue = particleSystem->getState();

    vector<Vector3f> f = particleSystem->evalF(currentValue);
    vector<Vector3f> f1;

    for(unsigned i=0; i<f.size(); ++i){
        Vector3f f1Value = currentValue[i] + stepSize * f[i];
        f1.push_back(f1Value);
    }

    vector<Vector3f> f_1 = particleSystem->evalF(f1);

    for(unsigned j = 0; j<currentValue.size(); ++j){
        Vector3f updateValue = currentValue[j] + (stepSize/2.0) * (f[j] + f_1[j]);
        newValue.push_back(updateValue);
    }

    particleSystem->setState(newValue);


}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 4.4
    vector<Vector3f> newValue;
    vector<Vector3f> currentValue = particleSystem->getState();

    vector<Vector3f> k_1 = particleSystem->evalF(currentValue);
    vector<Vector3f> k2;
    vector<Vector3f> k3;
    vector<Vector3f> k4;
    for(unsigned i = 0; i< k_1.size(); ++i){
        Vector3f k2_val = currentValue[i] + (stepSize/2.0) * k_1[i];
        k2.push_back(k2_val);
    }

    vector<Vector3f> k_2 = particleSystem->evalF(k2);

    for(unsigned j = 0; j< k_2.size(); ++j){
        Vector3f k3_val = currentValue[j] + (stepSize/2.0) * k_2[j];
        k3.push_back(k3_val);
    }

    vector<Vector3f> k_3 = particleSystem->evalF(k3);

    for(unsigned k = 0; k< k_3.size(); ++k){
        Vector3f k4_val = currentValue[k] + (stepSize * k_3[k]);
        k4.push_back(k4_val);
    }

    vector<Vector3f> k_4 = particleSystem->evalF(k4);

    for(unsigned x = 0; x < currentValue.size(); ++x){
        Vector3f updateValue = currentValue[x] + (stepSize/6.0) * (k_1[x] + 2 * k_2[x] + 2 * k_3[x] + k_4[x]);
        newValue.push_back(updateValue);
    }

    particleSystem->setState(newValue);



}

