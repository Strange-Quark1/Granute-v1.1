#include "physics.h"
#include "Vec2.h"
#include "entities.h"

void collision(ObjectManager& obj, int idA, int idB) {
    float dx = obj.posX[idB] - obj.posX[idA];
    float dy = obj.posY[idB] - obj.posY[idA];
    float distSq = dx*dx + dy*dy;
    float rsum = obj.radii[idA] + obj.radii[idB];
    
    if (distSq < rsum*rsum)
    {
        float dist = std::sqrt(distSq);
        float overlap = rsum - dist;
        float nx = dx / (dist>0 ? dist : 1); 
        float ny = dy / (dist>0 ? dist : 1); 
        float wA = obj.isFree[idA] / obj.masses[idA];
        float wB = obj.isFree[idB] / obj.masses[idB];
        float totalw = wA + wB;
        float pushX_A =  wA/(totalw>0?totalw:1) * overlap * nx;
        float pushY_A = wA/(totalw>0?totalw:1) * overlap * ny;
        float pushX_B = wB/(totalw>0?totalw:1) * overlap * nx;
        float pushY_B = wB/(totalw>0?totalw:1) * overlap * ny;
        obj.posX[idA] -= pushX_A;
        obj.posY[idA] -= pushY_A;
        obj.posX[idB] += pushX_B;
        obj.posY[idB] += pushY_B;
        obj.old_posX[idA] -= pushX_A;
        obj.old_posY[idA] -= pushY_A;
        obj.old_posX[idB] += pushX_B;
        obj.old_posY[idB] += pushY_B;
        float vxA = (obj.posX[idA] - obj.old_posX[idA])/obj.dt;
        float vyA = (obj.posY[idA] - obj.old_posY[idA])/obj.dt;
        float vxB = (obj.posX[idB] - obj.old_posX[idB])/obj.dt;
        float vyB = (obj.posY[idB] - obj.old_posY[idB])/obj.dt;
        float dvx = vxB - vxA;
        float dvy = vyB - vyA;
        float vn = dvx * nx + dvy * ny;
        if (vn > 0) return;
        float restitution = std::min(obj.restitution[idA], obj.restitution[idB]);
        float impulse = (-(1 + restitution) * vn) / (totalw > 0 ? totalw : 1);
        float impulseX = impulse * nx;
        float impulseY = impulse * ny;
        obj.old_posX[idA] += impulseX * wA* obj.dt;
        obj.old_posY[idA] += impulseY * wA* obj.dt;
        obj.old_posX[idB] -= impulseX * wB* obj.dt;
        obj.old_posY[idB] -= impulseY * wB* obj.dt;
    }
}

void stick(ObjectManager& obj, int idA, int idB, float d) {
    //float dist = (Vec2(obj.posX[idA], obj.posY[idA])-Vec2(obj.posX[idB],obj.posY[idB])).Magnitude();
    float dx = obj.posX[idB] - obj.posX[idA];
    float dy = obj.posY[idB] - obj.posY[idA];
    float dist = std::sqrt(dx*dx + dy*dy);
    float nx = dx / (dist>0 ? dist : 1); 
    float ny = dy / (dist>0 ? dist : 1); 

    float invmA = 1.0f / obj.masses[idA];
    float invmB = 1.0f / obj.masses[idB];
    float wA = obj.isFree[idA] * invmA;
    float wB = obj.isFree[idB] * invmB;
    float totalw = wA + wB;
    float pushX_A = wA / (totalw > 0 ? totalw : 1) * (dist - d) * nx * 0.05f;
    float pushY_A = wA / (totalw > 0 ? totalw : 1) * (dist - d) * ny * 0.05f;
    float pushX_B = wB / (totalw > 0 ? totalw : 1) * (dist - d) * nx * 0.05f;
    float pushY_B = wB / (totalw > 0 ? totalw : 1) * (dist - d) * ny * 0.05f;
    obj.posX[idA] += pushX_A;
    obj.posY[idA] += pushY_A;
    obj.posX[idB] -= pushX_B;
    obj.posY[idB] -= pushY_B;
}

void fix(ObjectManager& obj, int id)
{
    obj.isFree[id] = 0.0f;
}