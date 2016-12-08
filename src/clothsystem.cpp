#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"

 // your system should at least contain 8x8 particles.
const int W = 8;
const int H = 8;

using namespace std;
ClothSystem::ClothSystem()
{
    // TODO 5. Initialize m_vVecState with cloth particles. 
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting
    for (unsigned i = 0; i < H; ++i){
        for (unsigned j = 0; j < W; ++j){
            Vector3f pos = Vector3f(0.2*j - 0.8, 0, -0.2*i + 0.8 );
            Vector3f v = Vector3f(0,0,0);
            m_vVecState.push_back(pos);
            m_vVecState.push_back(v);
        }
    }
}

Vector3f ClothSystem::computeGravity(float mass){
    return mass * Vector3f(0,-9.8, 0);
}

Vector3f ClothSystem::computeDrag(float k, Vector3f velocity){
    return -k * velocity;
}

Vector3f ClothSystem::computeSpring(float k, float r, int icol, int irow, int jcol, int jrow, vector<Vector3f> pos) {
    if(icol < 0 || icol >= W || irow < 0 || irow >= H || jcol < 0 || jcol >= W || jrow < 0 || jrow >= H){
        return Vector3f(0,0,0);
    }

    int index_i = irow * W + icol;
    int index_j = jrow * W + jcol;

    Vector3f difference = pos[index_i] - pos[index_j];
    return -k * (difference.abs() - r) * difference.normalized();
}

Vector3f ClothSystem::computeStructString(int i, int j, const vector<Vector3f> & pos){
    float kStruct = 35;
    float r = 0.2;

    Vector3f top = computeSpring(kStruct, r, j, i, j, i-1, pos);
    Vector3f bot = computeSpring(kStruct, r, j, i, j, i+1, pos);
    Vector3f right = computeSpring(kStruct, r, j, i, j+1, i, pos);
    Vector3f left = computeSpring(kStruct, r, j, i, j-1, i, pos);

    return (top + bot) + (right + left);
}

Vector3f ClothSystem::computeShearString(int i, int j, const  vector<Vector3f> & pos){
    float kShear = 7.2;
    float r = 0.2;

    Vector3f topLeft = computeSpring(kShear, r, j, i, j-1, i-1, pos);
    Vector3f botLeft = computeSpring(kShear, r, j, i, j-1, i+1, pos);
    Vector3f topRight = computeSpring(kShear, r, j, i, j+1, i-1, pos);
    Vector3f botRight = computeSpring(kShear, r, j, i, j+1, i+1, pos);

    return (topLeft + botLeft) + (topRight + botRight);

}

Vector3f ClothSystem::computeFlexString(int i, int j, const vector<Vector3f>  & pos){
    float kFlex = 15;
    float r = 0.4;

    Vector3f top = computeSpring(kFlex, r, j, i, j, i-2, pos);
    Vector3f bot = computeSpring(kFlex, r, j, i, j, i+2, pos);
    Vector3f right = computeSpring(kFlex, r, j, i, j+2, i, pos);
    Vector3f left = computeSpring(kFlex, r, j, i, j-2, i, pos);

    return (top + bot) + (right + left);
}

vector<Vector3f> ClothSystem::getPositions(vector<Vector3f> state){
    vector<Vector3f> pos;
    for(unsigned i = 0;i<state.size(); i+=2){
        pos.push_back(state[i]);
    }
    return pos;
}

vector<Vector3f> ClothSystem::getVelocities(vector<Vector3f> state){
    vector<Vector3f> vel;
    for(unsigned i = 1; i<state.size(); i+=2){
        vel.push_back(state[i]);
    }
    return vel;
}

Vector3f ClothSystem::pickVel(int row, int col, vector<Vector3f> velocities){
    return velocities[row*W + col];
}
Vector3f ClothSystem::pickPos(int row, int col, vector<Vector3f>  positions){
    return positions[row*W + col];
}

std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
    float mass = 0.06;
    float drag = .11;


    vector<Vector3f> positions = getPositions(state);
    vector<Vector3f> velocities = getVelocities(state);
    std::vector<Vector3f> f;
    Vector3f allForces;

    for(unsigned i = 0; i < H; ++i){
        for(unsigned j = 0; j < W; j++){
            //pickVel(i,j,velocities).print();
            f.push_back(pickVel(i, j, velocities));
            //printf("%d %d\n", i, j);
            if ((i == 0 || i == H -1) && (j == 0 || j == W-1)) {
                allForces = Vector3f(0,0,0);
                //Vector3f allForces = computeGravity(mass) + computeDrag(drag, pickVel(i,j,velocities)) +
                //computeFlexString(i, j, positions) + computeShearString(i,j, positions) + computeStructString(i,j, positions);
            } else {
                allForces = (computeGravity(mass) + computeDrag(drag, pickVel(i, j, velocities)) +
                        computeStructString(i, j, positions) + computeShearString(i,j,positions) +
                        computeFlexString(i, j, positions))/(mass);
                //allForces.print();
            }
            f.push_back(allForces);
        }
    }
    // TODO 5. implement evalF
    // - gravity
    // - viscous drag
    // - structural springs
    // - shear springs
    // - flexion springs
     
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

    /*
    // EXAMPLE for how to render cloth particles.
    //  - you should replace this code.
    float w = 0.2f;
    Vector3f O(0.4f, 1, 0);
    gl.updateModelMatrix(Matrix4f::translation(O));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, 0, 0)));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, -w, 0)));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(0, -w, 0)));
    drawSphere(0.04f, 8, 8);
    */

    vector<Vector3f> positions = getPositions(getState());
    /*for(unsigned i = 0; i < positions.size(); ++i){
        gl.updateModelMatrix(Matrix4f::translation((positions[i])));
        drawSphere(0.04f, 8, 8);
    }*/
    
    const Vector3f N(0, 1, 0);
    VertexRecorder rec;

    gl.updateModelMatrix(Matrix4f::identity());
    for(unsigned i = 1; i < positions.size() - W; ++i){
	if(i % W != 0){
	    Vector3f p1 = positions[i-1];
	    Vector3f p2 = positions[i]; 
	    Vector3f p3 = positions[i+W-1];
	    Vector3f p4 = positions[i+W]; 
	    rec.record(p1, N);
    	    rec.record(p2, N);
            rec.record(p3, N);

            rec.record(p2, N);
            rec.record(p3, N);
            rec.record(p4, N);

	    rec.draw();
	}
    }

    /*
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
    rec.record(O, CLOTH_COLOR);
    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);

    rec.record(O, CLOTH_COLOR);
    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);

    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);

    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);
    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);
    glLineWidth(3.0f);
    rec.draw(GL_LINES);

    gl.enableLighting(); // reset to default lighting model
    // EXAMPLE END
     */

    /*
    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity());
    VertexRecorder rec;
    for(unsigned i = 0; i < H; ++i) {
        for (unsigned j = 0; j < W; j++) {
            if( i < H-1){
                rec.record(pickPos(i, j, positions), CLOTH_COLOR);
                rec.record(pickPos(i+1,j, positions), CLOTH_COLOR);
            }
            if(j < W - 1){
                rec.record(pickPos(i, j, positions), CLOTH_COLOR);
                rec.record(pickPos(i, j+1, positions), CLOTH_COLOR);
            }
        }
    }
    glLineWidth(3.0f);
    rec.draw(GL_LINES);
    gl.enableLighting(); // reset to default lighting model
    */
}

