#include "quadtree.h"
// constructor 
QuadTreeAllocator::QuadTreeAllocator(int maxNodes) {
    pool.resize(maxNodes); 
    nextAvailable = 0;
}

// Reset the allocator to reuse all nodes
void QuadTreeAllocator::reset() {
    nextAvailable = 0;
}

// Allocate a new quadtree node from the pool
QuadTree* QuadTreeAllocator::allocate(Boundary b, int cap) {
    QuadTree* node = &pool[nextAvailable++];
    node->qtreset(b, cap, this);
    return node;
}

void QuadTree::qtreset(Boundary b, int cap, QuadTreeAllocator* alloc) {
    boundary = b;
    //capacity = cap;
    allocator = alloc;
    currentPointCount = 0;
    divided = false;
    northwest = nullptr;
    northeast = nullptr;
    southwest = nullptr;
    southeast = nullptr;
}

void QuadTree::subdivide() {
    float x = boundary.x;
    float y = boundary.y;
    float w = boundary.w;
    float h = boundary.h;

    Boundary nw = {x - w/2, y - h/2, w/2, h/2};
    Boundary ne = {x + w/2, y - h/2, w/2, h/2};
    Boundary sw = {x - w/2, y + h/2, w/2, h/2};
    Boundary se = {x + w/2, y + h/2, w/2, h/2};
    northwest = allocator->allocate(nw, 4); 
    southwest = allocator->allocate(sw, 4);
    southeast = allocator->allocate(se, 4);
    northeast = allocator->allocate(ne, 4);

    for (int i = 0; i < currentPointCount; i++) {
        if (northwest->insert(points[i])) continue;
        if (northeast->insert(points[i])) continue;
        if (southwest->insert(points[i])) continue;
        if (southeast->insert(points[i])) continue;
    }
    currentPointCount = 0; // clear points from parent node after subdivision
    divided = true;
}

bool QuadTree::insert(Point p) {
    if (!boundary.contains(p)) return false;

    if (currentPointCount < 4 && !divided) {
        points[currentPointCount++] = p;
        return true; 
    }

    if (!divided) subdivide();

    if (northwest->insert(p)) return true;
    if (northeast->insert(p)) return true;
    if (southwest->insert(p)) return true;
    if (southeast->insert(p)) return true;

    return false;
}

void QuadTree::query(Boundary range , std::vector<Point>& found) {
    if (!boundary.intersects(range)) return; 

    for (int i = 0; i <currentPointCount; i++) {
        if (range.contains(points[i])){
            found.push_back(points[i]);
        }
    }
    if (divided) {
        northwest -> query (range, found);
        northeast -> query (range, found);
        southwest -> query (range, found);
        southeast -> query (range, found);
    }
    
    return;
}

void QuadTree::show() {
    DrawRectangleLines(
        (int)(boundary.x - boundary.w), 
        (int)(boundary.y - boundary.h), 
        (int)(boundary.w * 2), 
        (int)(boundary.h * 2), 
        WHITE
    );

    if (divided) {
        northwest->show();
        northeast->show();
        southwest->show();
        southeast->show();
    }
}