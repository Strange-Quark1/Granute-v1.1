#pragma once
#include "entities.h"
#include "Vec2.h"

void collision(ObjectManager& obj, int idA, int idB);
void stick(ObjectManager& obj, int idA, int idB, float d);
void rope(ObjectManager& obj, int idA, int idB, float length);