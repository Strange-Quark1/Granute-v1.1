#include "raylib.h"
#include "Vec2.h"
#include "entities.h"
#include "quadtree.h"
#include "physics.h"


int main() {
    InitWindow(800, 600, "Custom C++ Physics Engine");
    SetTargetFPS(120);
    
    ObjectManager manager;
    QuadTreeAllocator allocator(60000); // pre allocate 10000 quad tree nodes
    std::vector<Point> found;
    found.reserve(2000);
    //for (int i = 0; i < 4; i++){
        // manager.AddObject(
        //     (float)(rand() % 800), // x
        //     (float)(rand() % 600), // y
        //     (float)(rand() % 200 - 100) / 1.0f, // vx
        //     (float)(rand() % 200 - 100) / 1.0f, // vy
        //     0.0f, // ax
        //     0.0f, // ay
        //     1.0f, // mass
        //     3.0f, // radius
        //     1//(float)(rand() % 100) / 100.0f  // restitution
        // );
    // manager.AddObject(300-100, 200, 0, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);
    // manager.AddObject(400-100, 200, 0, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);
    // manager.AddObject(450-100, 286.602, 0, 0, 0, 400, 1.0f, 30.0f, 1.0f , 0.0f);
    // manager.AddObject(400-100, 373.204, 0, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);
    // manager.AddObject(300-100, 373.204, 0, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);
    // manager.AddObject(250-100, 286.602, 0, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);

    // manager.AddObject(300+150, 200, -400, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);
    // manager.AddObject(400+150, 200, -400, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);
    // manager.AddObject(450+150, 286.602, -400, 0, 400, 0, 1.0f, 30.0f, 1.0f , 1.0f);
    // manager.AddObject(400+150, 373.204, -400, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);
    // manager.AddObject(300+150, 373.204, -400, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);
    // manager.AddObject(250+150, 286.602, -400, 0, 0, 400, 1.0f, 30.0f, 1.0f, 1.0f);
    
    // for (int i = 1 ; i < 19 ; i++){
    //     manager.AddObject(
    //         200+i*20.0f, // x
    //         300.0f, // y
    //         0.0f, // vx
    //         0.0f, // vy
    //         0.0f, // ax
    //         400.0f, // ay
    //         3.0f, // mass
    //         9.0f, // radius
    //         1.0f,  // restitution
    //         (i==1 || i ==18)? 0.0f : 1.0f   // free
    //     );
    // }

    
    


    for (int i = 0 ; i < 10001; i++){
        manager.AddObject(
        (float)(rand() % 780 + 10), 
        (float)(rand() % 580 + 10), 
        (float)(rand() % 400 - 200) / 1.0f, // vx 
        (float)(rand() % 400 - 200) / 1.0f, // vy
        0.0f, // ax
        0.0f, // ay
        1.0f, // mass
        1.0f, // radius
        1.0f,  // restitution
        1.0f   // free
);
    }

    while (!WindowShouldClose()) {
        
        manager.Update();
            allocator.reset();
            Boundary boundary = {400, 300, 400, 300};
            QuadTree* qt = allocator.allocate(boundary, 32);
            for (int j = 0; j < manager.activeObjects; j++) {
                Point p = {j, manager.posX[j], manager.posY[j]};
                qt->insert(p);

            }
        for (int i = 0; i<4; i++)
        {

            
            for (int j = 0; j < manager.activeObjects; j++) {
                Boundary range = {manager.posX[j], manager.posY[j], manager.radii[j]*2, manager.radii[j]*2};
                found.clear();
                qt->query(range, found);
                for (const Point& other : found) {
                    if (other.id > j) {
                        collision(manager, j, other.id);
                        if (j < 18 && other.id < 18) continue;
                    }
                }
            }

            // stick(manager, 0, 1, 100.0f);
            // stick(manager, 1, 2, 100.0f);
            // stick(manager, 2, 3, 100.0f);
            // stick(manager, 3, 4, 100.0f);
            // stick(manager, 4, 5, 100.0f);
            // stick(manager, 5, 0, 100.0f);
            // stick(manager, 0, 3, 200.0f);
            // stick(manager, 1, 4, 200.0f);
            // stick(manager, 2, 5, 200.0f);
            // stick(manager, 1, 5, 173.205f);
            // stick(manager, 2, 4, 173.205f);

            // stick(manager, 6, 7, 100.0f);
            // stick(manager, 7, 8, 100.0f);
            // stick(manager, 8, 9, 100.0f);
            // stick(manager, 9, 10, 100.0f);
            // stick(manager, 10, 11, 100.0f);
            // stick(manager, 11, 6, 100.0f);
            // stick(manager, 6, 9, 200.0f);
            // stick(manager, 7, 10, 200.0f);
            // stick(manager, 8, 11, 200.0f);
            // stick(manager, 7, 11, 173.205f);
            // stick(manager, 8, 10, 173.205f);
            // for (int i = 0; i < 17; i++){
            //     stick(manager, i, i+1, 20.0f);
            // }
            manager.ApplyConstraints(800.0f, 600.0f);
        }


    

        BeginDrawing();
        ClearBackground(BLACK);

        // Draw the object 
        // for (int i = 0; i <= 5; i++) {
        //     Vec2 position(manager.posX[i], manager.posY[i]);
        //     DrawCircle((int)position.x, (int)position.y, manager.radii[i], BLUE);
        // }
        // DrawCircle((int)manager.posX[2], (int)manager.posY[2], manager.radii[2], GREEN);
        for (int i = 0; i < manager.activeObjects; i++) {
            Vec2 position(manager.posX[i], manager.posY[i]);
            DrawCircle((int)position.x, (int)position.y, manager.radii[i], RED);
        }
        // DrawLine((int)manager.posX[0], (int)manager.posY[0], (int)manager.posX[1], (int)manager.posY[1], WHITE);
        // DrawLine((int)manager.posX[1], (int)manager.posY[1], (int)manager.posX[2], (int)manager.posY[2], WHITE);
        // DrawLine((int)manager.posX[2], (int)manager.posY[2], (int)manager.posX[3], (int)manager.posY[3], WHITE);
        // DrawLine((int)manager.posX[3], (int)manager.posY[3], (int)manager.posX[4], (int)manager.posY[4], WHITE);
        // DrawLine((int)manager.posX[4], (int)manager.posY[4], (int)manager.posX[5], (int)manager.posY[5], WHITE);
        // DrawLine((int)manager.posX[5], (int)manager.posY[5], (int)manager.posX[0], (int)manager.posY[0], WHITE);
        // DrawLine((int)manager.posX[0], (int)manager.posY[0], (int)manager.posX[3], (int)manager.posY[3], WHITE);
        // DrawLine((int)manager.posX[1], (int)manager.posY[1], (int)manager.posX[4], (int)manager.posY[4], WHITE);
        // DrawLine((int)manager.posX[2], (int)manager.posY[2], (int)manager.posX[5], (int)manager.posY[5], WHITE);

        // DrawLine((int)manager.posX[6], (int)manager.posY[6], (int)manager.posX[7], (int)manager.posY[7], WHITE);
        // DrawLine((int)manager.posX[7], (int)manager.posY[7], (int)manager.posX[8], (int)manager.posY[8], WHITE);
        // DrawLine((int)manager.posX[8], (int)manager.posY[8], (int)manager.posX[9], (int)manager.posY[9], WHITE);
        // DrawLine((int)manager.posX[9], (int)manager.posY[9], (int)manager.posX[10], (int)manager.posY[10], WHITE);
        // DrawLine((int)manager.posX[10], (int)manager.posY[10], (int)manager.posX[11], (int)manager.posY[11], WHITE);
        // DrawLine((int)manager.posX[11], (int)manager.posY[11], (int)manager.posX[6], (int)manager.posY[6], WHITE);
        // DrawLine((int)manager.posX[6], (int)manager.posY[6], (int)manager.posX[9], (int)manager.posY[9], WHITE);
        // DrawLine((int)manager.posX[7], (int)manager.posY[7], (int)manager.posX[10], (int)manager.posY[10], WHITE);
        // DrawLine((int)manager.posX[8], (int)manager.posY[8], (int)manager.posX[11], (int)manager.posY[11], WHITE);


        DrawFPS(10, 30);
        EndDrawing();
    }
    
    CloseWindow();
    return 0;
}