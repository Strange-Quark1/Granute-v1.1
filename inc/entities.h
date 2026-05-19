#pragma once
#include "Vec2.h"
#include <vector>
class ObjectManager{
    public:
    std::vector<float> posX;
    std::vector<float> posY;
    std::vector<float> old_posX;
    std::vector<float> old_posY;
    std::vector<float> accX;
    std::vector<float> accY;
    std::vector<float> masses;
    std::vector<float> radii;
    std::vector<float> restitution;
    float dt;
    int activeObjects;
    int numObjects;
    std::vector<float> isFree;

    ObjectManager();

    void AddObject(
        float x,
        float y, 
        float vx,
        float vy,
        float ax,
        float ay,
        float m,
        float r, 
        float e,
        float free);
    void Update();
    void ApplyConstraints(float width, float height);
    
    void Pin(int id);
};