#include "entities.h"
#include <cmath>
#include <immintrin.h>
#include "Vec2.h"
#include <cstdio>

ObjectManager::ObjectManager(){
    int maxObjects = 10000;
    dt = 0.008f;
    activeObjects = 0;
    posX.resize(maxObjects, 0.0f);
    posY.resize(maxObjects, 0.0f);
    old_posX.resize(maxObjects, 0.0f); 
    old_posY.resize(maxObjects, 0.0f);
    accX.resize(maxObjects, 0.0f);
    accY.resize(maxObjects, 0.0f);
    masses.resize(maxObjects, 1.0f);
    radii.resize(maxObjects, 1.0f);     
    restitution.resize(maxObjects, 0.5f);
    isFree.resize(maxObjects, 1.0f);
}

void ObjectManager::AddObject(
        float x,
        float y, 
        float vx,
        float vy,
        float ax,
        float ay,
        float m,
        float r, 
        float e,
        float free) {
    //adding an object to the manager
    if (activeObjects < posX.size()){
        posX[activeObjects] = x;
        posY[activeObjects] = y; 
        old_posX[activeObjects] = x - vx*dt;
        old_posY[activeObjects] = y - vy*dt;   
        accX[activeObjects] = ax;
        accY[activeObjects] = ay;
        masses[activeObjects] = m;
        radii[activeObjects] = r;
        restitution[activeObjects] = e;
        isFree[activeObjects] = free;
        activeObjects++;}
    else{
        printf("Max objects reached! Cannot add more.\n");
    }
}

void ObjectManager::Pin(int id){
    isFree[id] = 0;
}

void ObjectManager::Update(){
    float dt_squared = dt * dt;
    __m128 dt2 = _mm_set1_ps(dt_squared);
    //__m128 two = _mm_set1_ps(2.0f);
    __m128 damping = _mm_set1_ps(0.999f);

    for (int i = 0; i < activeObjects; i+=4)
    {
        __m128 pX = _mm_loadu_ps(&posX[i]);//just the first memory address enoug
                                           //the next 3 values will be loaded as well since they are contiguous in memory
                                           //and the register is 128 bits wide (4 floats * 32 bits each = 128 bits)
                                           //SO COOL!! (I wrote this comment myself and is not a byproduct of AI btw just wanted to clarify that)
        __m128 pY = _mm_loadu_ps(&posY[i]);
        __m128 old_pX = _mm_loadu_ps(&old_posX[i]);
        __m128 old_pY = _mm_loadu_ps(&old_posY[i]);
        __m128 aX = _mm_loadu_ps(&accX[i]);
        __m128 aY = _mm_loadu_ps(&accY[i]);
        __m128 mask = _mm_loadu_ps(&isFree[i]);
        //verlet equation is p_new = p + damping(p - p_old) + a*dt^2
        // X component update
        __m128 pX_new = _mm_add_ps(_mm_add_ps(_mm_mul_ps(damping,_mm_sub_ps(pX, old_pX)),pX),_mm_mul_ps(aX, dt2)); 
        __m128 del_pX = _mm_sub_ps(pX_new, pX);
        pX_new = _mm_add_ps(pX, _mm_mul_ps(del_pX, mask)); 
        
        __m128 old_pX_new = pX;
        // Y component update
        __m128 pY_new = _mm_add_ps(_mm_add_ps(_mm_mul_ps(damping,_mm_sub_ps(pY, old_pY)),pY),_mm_mul_ps(aY, dt2));
        __m128 del_pY = _mm_sub_ps(pY_new, pY);
        pY_new = _mm_add_ps(pY, _mm_mul_ps(del_pY, mask));
        __m128 old_pY_new = pY;
        //store the new positions back to memory
        _mm_storeu_ps(&posX[i], pX_new);
        _mm_storeu_ps(&posY[i], pY_new);
        _mm_storeu_ps(&old_posX[i], old_pX_new);
        _mm_storeu_ps(&old_posY[i], old_pY_new);
    }

}

void ObjectManager::ApplyConstraints(float width, float height){
    //wall collision detection and response
    for (int i = 0; i < activeObjects; i++){
        float radius = radii[i];
        //left wall
        if (posX[i] - radius < 0){
            float vX = posX[i] - old_posX[i];
            posX[i] = radius;
            if (vX < 0)
            {
                old_posX[i] = posX[i] + vX * restitution[i];
            }
            else
            {

            old_posX[i] = posX[i] - vX;
            }
        }
        //right wall
        if (posX[i] + radius > width){
            float vX = posX[i] - old_posX[i];
            posX[i] = width - radius;
            if (vX > 0)
            {
                old_posX[i] = posX[i] + vX * restitution[i];
            }
            else
            {
                old_posX[i] = posX[i] - vX;
            }
        }
        //top wall
        if (posY[i] - radius < 0){
            float vY = posY[i] - old_posY[i];
            posY[i] = radius;
            if (vY < 0)
            {
                old_posY[i] = posY[i] + vY * restitution[i];
            }
            else
            {
                old_posY[i] = posY[i] - vY;
            }
        }
        //bottom wall
        if (posY[i] + radius > height){
            float vY = posY[i] - old_posY[i];   
            posY[i] = height - radius;
            if (vY > 0)
            {
                old_posY[i] = posY[i] + vY * restitution[i];
            }
            else
            {
                old_posY[i] = posY[i] - vY;
            }
        }
    }
}