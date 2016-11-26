//
// Created by John La on 11/26/16.
//

#include "rigidBall.h"
#include "camera.h"
#include "vertexrecorder.h"

RigidBall::RigidBall() {
    m_vVecState.push_back(Vector3f(1.0f, 0.0, 0.0));
}