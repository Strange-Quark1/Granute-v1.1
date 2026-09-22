# Granute — 2D High-Performance Verlet Physics Engine

Granute is a 2D physics simulation engine built in C++ and Raylib, designed to handle thousands of simultaneous particles and distance constraints in real time at 120+ FPS. The project prioritizes cache locality, minimal runtime heap allocations, and efficient spatial partitioning to maintain low memory and CPU overhead.

---

## Technical Architecture & Optimization Choices

### 1. Data Layout: Structure of Arrays (SoA)

Instead of an Array of Structures (AoS) approach (e.g., `std::vector<Particle>` where each object encapsulates its own properties), the engine uses a **Structure of Arrays (SoA)** design inside `ObjectManager`:

```cpp
// Contiguous, parallel vectors for CPU cache efficiency
std::vector<float> posX, posY;
std::vector<float> old_posX, old_posY;
std::vector<float> ax, ay;
std::vector<float> mass, radii, restitution, isFree;

```

* **Cache Locality:** During integration and collision sweeps, the CPU reads contiguous blocks of positions and velocities into L1/L2 cache lines without pulling in unrelated metadata.
* **Vectorization Potential:** Contiguous `float` arrays allow SIMD auto-vectorization across positional updates and constraint checks.
* **Memory Efficiency:** Avoids object alignment padding and per-object dynamic dispatch overhead.

---

### 2. Spatial Partitioning: QuadTree with Pre-Allocated Memory Pool

To avoid the naive $O(N^2)$ collision check cost ($100,000,000$ operations for $10,000$ particles), the engine uses a QuadTree broad-phase spatial partitioner.

* **Zero-Allocation Physics Loop (`QuadTreeAllocator`):** Standard dynamic quadtrees allocate and free nodes on the heap every frame using `new` and `delete`, causing severe memory fragmentation and allocator bottlenecks. Granute uses a pre-allocated pool buffer (e.g., `QuadTreeAllocator(60000)`) and resets the index pointer per frame (`allocator.reset()`), reducing node allocation cost to $O(1)$ pointer increments.
* **Broad-Phase Querying:** Spatial bounding-box queries (`qt->query(range, found)`) isolate candidate pairs into localized neighborhoods, pruning collision complexity down to $O(N \log N)$ on average.

---

### 3. Integration & Constraint Solving

#### Position-Based Verlet Integration

Position updating relies on Time-Corrected Verlet Integration:

$$x_{t + \Delta t} = 2x_t - x_{t - \Delta t} + a \cdot \Delta t^2$$

* **Implicit Velocity Storage:** Velocity is implicitly represented by $(x_t - x_{t - \Delta t})$. This eliminates explicit velocity tracking vectors and naturally handles drag/external positioning by updating `old_posX` alongside `posX`.
* **Stability in Distance Constraints:** Verlet integration pairs naturally with Position-Based Dynamics (PBD) for stick/spring constraints.

#### Iterative Constraint Solver

To maintain rigidity across connected links and boundary collisions without explosive kinetic instabilities, the engine runs $K$ relaxation iterations per frame ($K = 4$):

```cpp
for (int i = 0; i < 4; i++) {
    // 1. Broad-phase QuadTree query + pairwise collision response
    // 2. Distance constraint (stick) adjustment
    // 3. Boundary constraint enforcement
}

```

Multi-passing over distance and collision constraints enforces convergence toward rigid link lengths and reduces visual elasticity.

---

## Memory & Performance Profile

Below is a breakdown of resource usage under a standard 10,000-particle load test:

| Metric | Measured Value | Notes |
| --- | --- | --- |
| **Total RAM Footprint** | ~117 MB | Includes OpenGL context, Raylib swapchain, UI buffers, and pre-allocated QuadTree pool. |
| **Active Particle Data** | ~400 KB | $10,000 \text{ particles} \times 10 \text{ float attributes} \times 4 \text{ bytes}$. |
| **QuadTree Node Pool** | ~2.9 MB | $60,000 \text{ pre-allocated nodes}$. |
| **CPU Usage** | ~7% (1 core) | Single-threaded frame calculation finishing well within the $8.33\text{ ms}$ budget ($120\text{ FPS}$). |
| **Runtime Heap Allocations** | $0\text{ bytes/frame}$ | All particle arrays and QuadTree nodes reuse pre-allocated capacity. |

---

## Features & Controls

* **Spawn Mode:** Interactive particle generation with configurable mass, radius, restitution (bounce coefficient), and static/pinned state.
* **Link Mode (Distance Constraints):** Connect any two particles with rigid or elastic sticks with adjustable stiffness parameters.
* **Drag Mode:** Real-time user manipulation of live particles. Synchronizes current position with Verlet history buffers (`old_posX`, `old_posY`) to prevent momentum explosion on release.
* **Right-Click Pinning:** Contextual toggle to convert active dynamic particles into static structural anchors (and vice-versa) directly in the viewport.
* **Real-time Performance Metrics:** Live display of FPS and Private Working Set memory consumption.

---

## Build Instructions

### Prerequisites

* C++17 compatible compiler (GCC/MinGW, Clang, or MSVC)
* CMake 3.15+
* Raylib 4.x / 5.x
* Raygui

### Compilation

```bash
# Clone the repository
git clone https://github.com/your-username/granute.git
cd granute

# Generate build files
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Compile
cmake --build build --config Release

```

### Executable Output

The compiled executable will be located in `./build/PhysicsEngine` or `./build/Release/PhysicsEngine.exe`.
