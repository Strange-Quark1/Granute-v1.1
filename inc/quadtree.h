#pragma once

#include <vector>
#include "raylib.h"

struct Point {int id; float x; float y; }; // Point structure for 2D space

struct Boundary { // Rectangle boundary
    float x, y, w, h;
    bool contains(Point p) const {
        return (p.x >= x - w && p.x <= x + w && p.y >= y - h && p.y <= y + h);  // Check if point is within boundary
    }
    bool intersects(Boundary range) const {
        return !(range.x - range.w > x + w || range.x + range.w < x - w ||
                 range.y - range.h > y + h || range.y + range.h < y - h);
    } // Check if boundaries intersect
    //bool totalintersects(Boundary range);
};

class QuadTree; // Declaration for QuadTree to be used in QuadTreeAllocator


// QuadtreeAllocator contains a pool of quadtrees and allocates them to each node as needed in O(1) time. 
// It also has a reset function that "deletes" all nodes in O(1) time by simply resetting the index.
class QuadTreeAllocator {
public:
    std::vector<QuadTree> pool;
    int nextAvailable;
    QuadTreeAllocator(int maxNodes);
    void reset();
    QuadTree* allocate(Boundary b, int cap); // pointer is used to avoid copying entire quadtree when inserting into the pool
};


class QuadTree {
public:
    Boundary boundary;
    //int capacity=4;
    Point points[4];
    bool divided;

    QuadTree* northwest;
    QuadTree* northeast;
    QuadTree* southwest;
    QuadTree* southeast;

    QuadTreeAllocator* allocator;

    QuadTree() = default; 
    QuadTree(Boundary b, int cap, QuadTreeAllocator* alloc);

    void qtreset(Boundary b, int cap, QuadTreeAllocator* alloc);
    int currentPointCount;
    void subdivide();
    bool insert(Point p);
    void query(Boundary range, std::vector<Point>& found);
    void show();
};