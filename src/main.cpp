#include "raylib.h"
#include "Vec2.h"
#include "entities.h"
#include "quadtree.h"
#include "physics.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

int GetClickedParticle(ObjectManager &manager, Vector2 mousePos)
{
    for (int i = 0; i < manager.activeObjects; i++)
    {
        float dx = manager.posX[i] - mousePos.x;
        float dy = manager.posY[i] - mousePos.y;
        if (dx * dx + dy * dy <= manager.radii[i] * manager.radii[i])
        {
            return i;
        }
    }
    return -1;
}

int main()
{
    InitWindow(800, 600, "granute");
    SetTargetFPS(120);

    ObjectManager manager;
    QuadTreeAllocator allocator(60000); // pre allocate 10000 quad tree nodes
    std::vector<Point> found;
    found.reserve(2000);

    Rectangle uiBounds = {10, 10, 180, 230};
    int currentMode = 0; // 0: spawn, 1: stick
    float uiRadius = 10.0f;
    float uiMass = 1.0f;
    float uiRestitution = 0.5f;
    bool uiPinned = false;
    float uiStiffness = 0.05f;
    int selectedLinkA = -1;
    int draggedParticle = -1; 

    for (int i = 0; i < 5001; i++)
    {
        manager.AddObject(
            (float)(rand() % 780 + 10),
            (float)(rand() % 580 + 10),
            (float)(rand() % 400 - 200) / 1.0f, // vx
            (float)(rand() % 400 - 200) / 1.0f, // vy
            0.0f,                               // ax
            0.0f,                               // ay
            1.0f,                               // mass
            1.0f,                               // radius
            1.0f,                               // restitution
            1.0f                                // free
        );
    }

    while (!WindowShouldClose())
    {

        Vector2 mousePos = GetMousePosition();
        bool overUI = CheckCollisionPointRec(mousePos, uiBounds);

        // Handle Mouse Clicks (only if not clicking the UI panel)
        if (!overUI && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            if (currentMode == 0)
            {
                // spawn (ay = 400 for gravity)
                manager.AddObject(mousePos.x, mousePos.y, 0, 0, 0, 400.0f, uiMass, uiRadius, uiRestitution, uiPinned ? 0.0f : 1.0f);
            }
            else if (currentMode == 1)
            {
                // link
                int clickedId = GetClickedParticle(manager, mousePos);
                if (clickedId != -1)
                {
                    if (selectedLinkA == -1)
                    {
                        selectedLinkA = clickedId; 
                    }
                    else if (selectedLinkA != clickedId)
                    {
                        manager.AddStick(selectedLinkA, clickedId, -1.0f); 
                        selectedLinkA = -1;
                    }
                }
                else
                {
                    selectedLinkA = -1;
                }
            }

            else if (currentMode == 2)
            {
               
                draggedParticle = GetClickedParticle(manager, mousePos);
            } 
        } 
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && draggedParticle != -1)
        {
            manager.posX[draggedParticle] = mousePos.x;
            manager.posY[draggedParticle] = mousePos.y;
            manager.old_posX[draggedParticle] = mousePos.x;
            manager.old_posY[draggedParticle] = mousePos.y;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            draggedParticle = -1; 
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
        {
            int clickedId = GetClickedParticle(manager, mousePos);
            if (clickedId != -1)
            {
                manager.isFree[clickedId] = !manager.isFree[clickedId]; 
            }
        }
    

    manager.Update();
    allocator.reset();
    Boundary boundary = {400, 300, 400, 300};
    QuadTree *qt = allocator.allocate(boundary, 32);
    for (int j = 0; j < manager.activeObjects; j++)
    {
        Point p = {j, manager.posX[j], manager.posY[j]};
        qt->insert(p);
    }
    for (int i = 0; i < 4; i++)
    {

        for (int j = 0; j < manager.activeObjects; j++)
        {
            Boundary range = {manager.posX[j], manager.posY[j], manager.radii[j] * 2, manager.radii[j] * 2};
            found.clear();
            qt->query(range, found);
            for (const Point &other : found)
            {
                if (other.id > j)
                {
                    collision(manager, j, other.id);
                    if (j < 18 && other.id < 18)
                        continue;
                }
            }
        }
        for (int s = 0; s < manager.activeSticks; s++)
        {
            stick(manager, manager.stickA[s], manager.stickB[s], manager.stickLength[s], uiStiffness);
        }

        manager.ApplyConstraints(800.0f, 600.0f);
    }

    BeginDrawing();
    ClearBackground(BLACK);
    for (int s = 0; s < manager.activeSticks; s++)
    {
        DrawLineEx(
            (Vector2){manager.posX[manager.stickA[s]], manager.posY[manager.stickA[s]]},
            (Vector2){manager.posX[manager.stickB[s]], manager.posY[manager.stickB[s]]},
            2.0f, LIGHTGRAY);
    }

    for (int i = 0; i < manager.activeObjects; i++)
    {
        Vec2 position(manager.posX[i], manager.posY[i]);
        // pinned objects green, free objects red
        Color col = manager.isFree[i] == 0.0f ? GREEN : RED;
        DrawCircle((int)position.x, (int)position.y, manager.radii[i], col);
    }

    if (currentMode == 1 && selectedLinkA != -1)
    {
        DrawCircleLines((int)manager.posX[selectedLinkA], (int)manager.posY[selectedLinkA], manager.radii[selectedLinkA] + 4, YELLOW);
        DrawLine((int)manager.posX[selectedLinkA], (int)manager.posY[selectedLinkA], mousePos.x, mousePos.y, YELLOW); // Draw string to mouse
    }

    GuiPanel(uiBounds, "granute");

    // Mode Switcher
    GuiToggleGroup((Rectangle){15, 35, 50, 20}, "SPAWN;LINK;DRAG", &currentMode);

    if (currentMode == 0)
    {
        // Tighter vertical spacing and thinner sliders
        GuiLabel((Rectangle){15, 65, 50, 15}, "Radius:");
        GuiSliderBar((Rectangle){65, 65, 110, 15}, "", TextFormat("%.1f", uiRadius), &uiRadius, 2.0f, 40.0f);

        GuiLabel((Rectangle){15, 90, 50, 15}, "Mass:");
        GuiSliderBar((Rectangle){65, 90, 110, 15}, "", TextFormat("%.1f", uiMass), &uiMass, 0.1f, 10.0f);

        GuiLabel((Rectangle){15, 115, 50, 15}, "Bounce:");
        GuiSliderBar((Rectangle){65, 115, 110, 15}, "", TextFormat("%.2f", uiRestitution), &uiRestitution, 0.0f, 1.0f);

        GuiCheckBox((Rectangle){15, 140, 15, 15}, "Pinned", &uiPinned);
    }
    else if (currentMode == 1)
    {
        GuiLabel((Rectangle){15, 65, 150, 15}, selectedLinkA != -1 ? "Select second particle." : "Select first particle.");
        GuiLabel((Rectangle){15, 90, 60, 15}, "Stiffness:");
        GuiSliderBar((Rectangle){75, 90, 100, 15}, "", TextFormat("%.2f", uiStiffness), &uiStiffness, 0.01f, 1.0f);
    }

    else if (currentMode == 2)
    {
        GuiLabel((Rectangle){15, 65, 150, 15}, "Drag particles with mouse.");
    }

    // Tighter buttons at the bottom
    if (GuiButton((Rectangle){15, 170, 160, 20}, "Clear Screen"))
    {
        manager.activeObjects = 0;
        manager.activeSticks = 0;
        selectedLinkA = -1;
    }

    if (GuiButton((Rectangle){15, 195, 160, 20}, "Drop Cluster (x50)"))
    {
        for (int i = 0; i < 50; i++)
        {
            manager.AddObject(GetRandomValue(300, 500), GetRandomValue(50, 150), 0, 0, 0, 400.0f, uiMass, uiRadius, uiRestitution, 1.0f);
        }
    }

    DrawFPS(10, 30);
    EndDrawing();
}

CloseWindow();
return 0;
}
