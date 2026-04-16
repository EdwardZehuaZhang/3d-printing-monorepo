# Single Wire Sensing: Research Paper Implementation Explained
## A Tale of Two Versions (Legacy v30 & v3.5 Structure)

**Prepared for: Professor Presentation**  
**Goal:** Understand how single-wire sensing works, and key improvements from legacy to modern versions

---

## Table of Contents
1. [What is Single-Wire Sensing?](#what-is-single-wire-sensing) - The Big Picture
2. [Legacy v30 Approach](#legacy-v30-approach---foundational-research)
3. [V3.5 Modern Approach](#v35-modern-approach---optimized-implementation)
4. [Key Architectural Differences](#key-architectural-differences)
5. [Visual Comparison](#visual-comparison)

---

## What is Single-Wire Sensing?

### The Problem We're Solving
Imagine you want to **detect touch on multiple points** of a 3D-printed object. Normally, you'd need one wire per touch point. That's expensive and complex. **Single-wire sensing solves this with ONE continuous conductive wire.**

### How It Works (Simple Explanation)

**The Basic Circuit:**
```
Battery (5V) → Main Wire (R0, wire resistance) → 
    Touch Node 1 (connects to trace) → 
    Trace Network (R1) → 
    Touch Node 2 → 
    ... more touch nodes ...
    → Back to Battery
```

**The Magic - Time-Based Position Detection:**
- When you **touch the wire at Node 1**, you create a **capacitive coupling** to a sensor
- The closer the touch is to the battery, the **faster** the voltage reaches your sensor threshold
- Each physical location has a **unique response time** 
- By measuring response times, you can **identify WHICH touch node was pressed**

**Why One Wire Works:**
- The wire itself forms the **topology** of the network
- Resistance varies along the wire's path
- Each touch point has a **unique electrical signature** (different resistance values between nodes)

### Key Insight from Research
The paper optimizes **resistance values** (R0 = main wire, R1 = trace resistance) so that:
- Touch responses are **maximally separated in time**
- Slowest touch and fastest touch responses are **well-spaced** (good discrimination)
- All responses happen **quickly enough** (within ~1ms timeout)

---

## Legacy v30 Approach - Foundational Research

### Architecture Overview

```
v30 Structure:
├── Contour-based path finding
├── Basic resistance optimization
└── Simplified node-to-node routing
```

### Core Algorithm Flow

**Step 1: Create Surface Contours**
```
Input: 3D mesh object
↓
Generate nested contour layers (like slicing an onion)
↓
Use contour surfaces as guidance for routing paths
```

**Step 2: Route Paths Between Touch Nodes**
- **Method:** Contour-based path finding
- **How:** 
  1. Start from touch node A
  2. Ray-cast toward outer boundary to find safe points
  3. Walk along contour layers between A and B
  4. Alternate direction to minimize path length
  5. End at touch node B

**Step 3: Optimize Wire Properties**
```python
def optimize_resistances(n_nodes):
    """
    Find best R0 (main wire) and R1 (trace resistance)
    Challenge: Make all touch points *distinguishable*
    
    For worst-case timing:
    - SLOWEST touch: path goes through entire resistance network first
    - FASTEST touch: path touches near battery
    
    Goal: Maximize time difference between slow and fast
    """
    
    # Try all combinations of R0 and R1
    for r0 in wire_resistances:
        for r1 in trace_resistances:
            # Simulate circuit with symbolic math (lcapy)
            circuit = build_circuit(r0, r1)
            
            # When touch happens at node i:
            # - Measure voltage rise time to threshold (2.5V)
            
            # Score: time_slow - time_fast
            # Higher score = better discrimination
```

### Key Characteristics of v30

| Aspect | Implementation |
|--------|-----------------|
| **Path Finding** | Contour-based (uses surface topology) |
| **Resistance Calc** | Based on vertical/horizontal resistivity |
| **Node Routing** | Sequential node-to-node paths |
| **Optimization** | Brute-force grid search of (R0, R1) |
| **Geometry** | Straight-line traces between contours |
| **Focus** | Research validation, simple rules |

### Code Structure (Conceptual)

```python
# Main workflow in v30 (pseudo-code)

# 1. Load mesh and select touch nodes
mesh = load_mesh("bunny.stl")
touch_nodes = select_points_on_surface(mesh)

# 2. Generate contours (nested layers inside mesh)
contours = generate_nested_contours(mesh, num_layers=10)

# 3. Find paths using contours
for start_node, end_node in node_pairs:
    path = contour_based_path_finding(
        start_node, 
        end_node, 
        contours
    )

# 4. Optimize circuit resistances
r0, r1, quality = optimize_resistances(
    n_nodes=len(touch_nodes),
    wire_resistance_range=(0.1M - 10M Ohms),
    trace_resistance_range=(100k - 500k Ohms)
)

# 5. Generate 3D geometry for printing
geometry = generate_wire_geometry(paths, r0, r1)
geometry.save("bunny_sensing.stl")
```

### Advantages of v30
✅ Respects surface topology  
✅ Simple conceptual model  
✅ Works for convex objects  
✅ Clear mapping: contours → paths  

### Limitations of v30
❌ Can fail on complex topologies  
❌ Doesn't handle interior obstacles  
❌ Inefficient for large spaces  
❌ No real spacing guarantees  
❌ Path quality depends on contour layout  

---

## V3.5 Modern Approach - Optimized Implementation

### Architecture Overview

```
v3.5 Structure (Rhino-based):
├── 3D Voxel Grid Creation (with spacing)
├── Graph-based A* pathfinding
├── Advanced detour strategies
├── Roominess-guided optimization
├── Serpentine fill for length adjustment
└── Multi-segment spacing enforcement
```

### Core Algorithm Innovation

**Key Insight:** Instead of following surface contours, create a **3D grid** inside the object and use **A* pathfinding** with advanced strategies.

### Step-by-Step Process

**Step 1: Build Valid 3D Grid**
```
Input: Closed mesh from Rhino
↓
For each point in 3D space:
  - Is it inside the mesh? ✓
  - Is it far enough from boundary (clearance)? ✓
  - Is it far enough from previous routes? ✓
  
↓
Result: Set of "valid cells" where wire CAN go
```

**Step 2: Create Roominess Map**
```python
def build_roominess_map(valid_cells):
    """
    For each cell, count neighbors in 3×3×3 box
    Roominess = number of valid neighbors
    
    Purpose: Identify tight vs open spaces
    """
    roominess = {}
    for cell in valid_cells:
        # Count neighbors within radius=2
        neighbors = neighbors_in_radius(cell, radius=2)
        roominess[cell] = count(neighbors in valid_cells)
    
    return roominess

# Cells with high roominess: plenty of space
# Cells with low roominess: tight bottlenecks
```

**Step 3: Route Node Sequence with Advanced Strategies**

For each pair of touch nodes (start → target):

**Strategy 1: Direct A* Path**
```
Use classic A* algorithm:
- Cost = distance traveled
- Heuristic = straight-line to target
- Avoid cells marked as "blocked"
- Result: Shortest valid path
```

**Strategy 2: Serpentine Fill (if path is too short)**
```
If target_length > direct_path × 1.5:
    
    1. Build corridor around direct path
    2. Divide space into layers & rows
    3. Snake back-and-forth through layers
    4. Adjust spacing based on roominess
    
    Result: Longer path filling interior space
```

**Strategy 3: Detour-Based Path Growth**
```
If still need more length:
    
    1. Create "waypoint" candidates (high-roominess cells)
    2. Try routing: start → waypoint1 → waypoint2 → target
    3. Greedily add waypoints until target length reached
    
    Result: Intelligently extended path
```

**Step 4: Enforce Multi-Segment Spacing**
```
After routing first segment (node 1 → node 2):
  - Mark that path as "occupied"
  - Block all cells within `blocked_radius` distance
  - Next segment must route around it
  - Exception zones allow connection to next touch node

Effect: Parallel, non-intersecting wires
```

**Step 5: Generate Conductive Geometry**
```
For each path (list of waypoints):
    - Connect consecutive points with cylinders (wire diameter)
    - Add spheres at junctions for smooth transitions
    - Union into single solid object
    
Output: 3D geometry ready for 3D printing
```

### Visual of v3.5 Workflow

```
Closed Mesh Input
    ↓
[Build 3D Grid] ← Remove cells too close to boundary
    ↓
    ↓ 
    ├─→ [A* Direct Path] → Path A
    │
    ├─→ [Serpentine Fill] → Path B
    │
    └─→ [Detour Waypoints] → Path C
    
    [Choose Best Path for Target Length]
    ↓
    [Mark Segment as Blocked]
    ↓
    [Route Next Segment Avoiding Block]
    ↓
    [Repeat for all node pairs]
    ↓
    [Generate 3D Geometry]
    ↓
Output: Conductive wire pathway solid
```

### Key v3.5 Code Components

```python
# 1. Grid creation with clearance
valid_cells = build_valid_grid(
    mesh=mesh,
    wire_diameter_mm=0.5,
    casing_thickness_mm=0.5,
    boundary_clearance_mm=0.5
)
# Result: Only cells far enough from boundaries

# 2. Roominess map for optimization
roominess = build_roominess_map(valid_cells, radius=2)

# 3. Route node sequence with constraints
paths = route_node_sequence(
    waypoints=touch_nodes,
    valid_cells=valid_cells,
    target_lengths=[2000, 1500, 1800],  # mm per segment
    blocked_radius=3,  # cells
    spacing_radius=2   # cells
)

# 4. Generate 3D geometry
geometry = segment_solids(paths, wire_radius=0.25)
```

### Comparison Table: v30 vs v3.5

| Feature | v30 | v3.5 |
|---------|-----|------|
| **Grid Type** | Surface contours | 3D voxel grid |
| **Pathfinding** | Contour walking | A* with detours |
| **Spacing Control** | Implicit (surface-based) | Explicit (blocked_radius) |
| **Length Adjustment** | Resistance optimization only | Spirals, serpentines, waypoints |
| **Multi-segment Isolation** | Not enforced | Explicit blocking zones |
| **Open Space Support** | Limited | Excellent |
| **Complex Topology** | May fail | Robust |
| **Computational Speed** | Faster | More thorough |
| **Code Complexity** | Simpler | More sophisticated |

---

## Key Architectural Differences

### 1. **Path Representation**

**v30:** Contour layers
```
Mesh outer surface
    ↓ (inward slicing)
Contour 1 (outermost)
    ↓
Contour 2
    ↓
Contour 3 (innermost)

Paths constrained to these surfaces
```

**v3.5:** 3D voxel grid
```
Uniform 3D space divided into cubic cells
Every valid cell is potential routing location
Pathfinding explores all valid cells equally
```

### 2. **Routing Algorithm**

**v30:**
```python
# Contour-based routing
path = []
current_contour_index = last  # Start at outer contour
while not at_target:
    # Find closest point on next inner contour
    next_point = find_closest_on_contour(current_contour_index - 1)
    path.append(next_point)
    current_contour_index -= 1
```

**v3.5:**
```python
# A* with intelligent detours
path = astar_search(
    start=start_node,
    goal=target_node,
    heuristic=euclidean_distance,
    blocked_cells=blocked_set
)

# If path too short:
path = serpentine_fill(path, target_length)
# OR
path = detour_waypoint_routing(path, target_length)
```

### 3. **Spacing Between Segments**

**v30:**
```
Segment 1: Node A → Node B (contour path)
Segment 2: Node B → Node C (separate contour path)

✓ Naturally separated (different contour layers)
✗ May touch at junction points
```

**v3.5:**
```
Segment 1: Node A → Node B (A* path)
    ↓ Mark all cells within blocked_radius as occupied
Segment 2: Node B → Node C (A* avoids blocked cells)
    ↓ Separate by explicit distance constraint

✓ Guaranteed minimum spacing
✓ Controlled via blocked_radius parameter
```

### 4. **Length Adjustment Strategy**

**v30:**
```
Given fixed paths from v30 routing:
- Calculate resulting resistances
- Adjust R0 (main wire) and R1 (trace resistance)
- Optimize for time discrimination
- Path lengths are FIXED
```

**v3.5:**
```
Given target length for each segment:
1. Try direct A* path
   - If too long: done ✓
   - If too short: try next strategy
   
2. Try serpentine fill
   - Zigzag back-and-forth through space
   - Fill interior volume
   - If achieves target: done ✓
   
3. Try waypoint detours
   - Route through intermediate waypoints
   - Greedy expansion toward target
   - If achieves target: done ✓
   
- Paths are ADAPTIVE to target requirements
```

### 5. **Error Handling**

**v30:**
- Contours may not exist in all regions
- Path may fail if impossible to connect
- **Limited strategy for recovery**

**v3.5:**
- Multiple fallback strategies
- Can expand search space gradually
- **Sophisticated error recovery**

---

## Visual Comparison

### v30: Contour-Based (Layer Slicing)

```
        Outer Surface (Mesh)
              ╱╲
            ╱    ╲
         Node 1  Node 2
          │         │
        ╱─┼─────────┼─╲
       │  Contour 1  │ (Outer layer)
       │   ╱──────╲  │
       │  │Contour 2│ (Middle)
       │  │╱──────╲ │ │
       │ │└─Contour─┘│ │ (Inner)
       │ └─────────────┘ │
       └─────────────────┘

Routing:
- Follow contours like highways
- Nested layers guide paths
- Simple but inflexible
```

### v3.5: Voxel-Grid Based (Space Filling)

```
         Mesh Boundary
        ┌─────────────┐
        │             │
        │ ✓ ✓ ✓ ✓ ✓ │ Valid cells
        │ ✓ ✓ ✗ ✓ ✓ │ (✓ = inside, ✗ = blocked)
        │ ✓ X X Y ✓ │ Paths X and Y maintained
        │ ✓ ✓ ✓ ✓ ✓ │ separate by blocked_radius
        │ ✓ ✓ ✓ ✓ ✓ │
        └─────────────┘

Routing:
- Grid cells as nodes
- A* explores all possibilities
- Detours and spirals available
- Flexible and robust
```

---

## Practical Comparison: Bunny Model

### v30 Implementation
```
Input: Bunny mesh + 8 touch nodes

Step 1: Generate contours (slices through bunny)
        Result: ~15 nested contours

Step 2: Route between each node pair
        Node 1 → Node 2: Follow contours, 300mm path
        Node 2 → Node 3: Follow contours, 280mm path
        ...

Step 3: Optimize resistances
        Best R0 = 1.9 MΩ (main wire)
        Best R1 = 150 kΩ (per trace)
        
        Time discrimination:
        Slowest touch: 0.95ms
        Fastest touch: 0.55ms
        Delta: 0.40ms ✓ (sufficient)

Output: 3D bunny with internal wire
        (bunny_sensing.stl)
```

### v3.5 Implementation
```
Input: Bunny mesh + 8 touch nodes

Step 1: Build 3D voxel grid (step=0.5mm)
        Result: ~500K valid cells

Step 2: Build roominess map
        Identify tight spots vs open spaces

Step 3: Route with target lengths
        Node 1 → Node 2: 
            - Direct A* = 250mm (too short)
            - Add serpentine = 450mm ✓ Target reached
        Node 2 → Node 3:
            - Direct A* = 290mm (too long) ✓
        ...

Step 4: Enforce spacing
        Segment 1 blocks radius-2 around it
        Segment 2 routes around blocked cells
        Result: Parallel, non-intersecting paths

Step 5: Generate geometry
        Cylinders + sphere junctions
        Union into single solid

Output: 3D bunny with controlled internal wire
        (bunny_optimized_v3.5.stl)
        Better length control, guaranteed spacing
```

---

## Why These Differences Matter

### v30 (Academic Foundation)
**Best for:**
- Understanding the research principles
- Simple convex shapes
- Validating the core sensing concept
- Historical reference

**Challenges:**
- Can't handle complex interior paths
- Limited control over path lengths
- No spacing guarantees between parallel routes

### v3.5 (Practical Implementation)
**Best for:**
- Complex 3D geometries
- Precise control over conductive path properties
- Manufacturing-ready designs
- Production quality

**Advantages:**
- Robust pathfinding in any topology
- Guaranteed non-intersecting routes
- Adaptive length control (spirals, serpentines)
- Better handling of tight spaces

---

## Summary: The Evolution

| Generation | Focus | Method | Result |
|-----------|-------|--------|--------|
| **Research (v30)** | Proof of concept | Surface contours | Validation |
| **Production (v3.5)** | Robust implementation | Voxel grid + A* | Manufacturing |

### Key Takeaway
**v30** proved that one wire with optimized resistances CAN detect multiple touch points through timing discrimination. **v3.5** solved the practical engineering problem of reliably routing that wire through complex 3D geometries with guaranteed spacing and control.

---

## Questions for Discussion

1. **Why can one wire detect multiple touch points?**
   - Answer: Each location has unique electrical signature (resistance values between nodes create unique RC time constant)

2. **Why is spacing between routes important?**
   - Answer: Prevents crosstalk and electrical shortcutting between the main wire and trace network

3. **What makes v3.5 more robust than v30?**
   - Answer: Multiple routing strategies allow fallback to complex geometries; explicit spacing gives guarantees

4. **How does length adjustment work in v3.5?**
   - Answer: Serpentine fill and waypoint detours intelligently expand paths to target length by spiraling and detouring

5. **Why optimize both R0 and R1?**
   - Answer: R0 (main wire) and R1 (trace network) together determine the RC circuit's time constants; optimization maximizes time separation between fastest and slowest touch responses

---

## Technical Deep Dive (Optional Appendix)

### Circuit Analysis

**The Circuit Model:**
```
5V Battery
   ├── R0 (Main wire)
   ├── Node 1 (Touch input)
   ├── R1 (Trace between nodes)
   ├── Node 2
   ├── R1 (Trace between nodes)
   └── ... repeats for each node
   └── Back to battery through high-impedance sensor

When you touch at Node i:
- Current flows: Battery → R0 → through touched node → traces → sensor
- Time to reach 2.5V threshold depends on:
  - Distance from battery (affects series R)
  - Number of trace resistances in path
```

### Why Time Is Different Per Location

```
Touch at Node 1 (closest to battery):
  Resistance path = R0 only ≈ short time
  
Touch at Node 5 (middle):
  Resistance path = R0 + 2×R1 ≈ medium time
  
Touch at Node 8 (farthest):
  Resistance path = R0 + 4×R1 ≈ long time
  
Key: R0 and R1 optimized so time differences are LARGE
     and well-separated for easy discrimination
```

### Resistance Optimization Details

```python
def optimize_resistances(n_nodes=8):
    """
    Constraint: Find R0 and R1 such that
    - Slowest touch time - Fastest touch time is maximum
    - All times < 1ms (timeout)
    - All initial voltages at t=0 < 2.5V threshold
    
    The "slowest" scenario: touch at farthest node
    - Entire series R in path: R0 + (n_nodes-1)×R1
    
    The "fastest" scenario: touch at first node  
    - Minimal R: R0
    
    Goal: Maximize time separation while staying in constraints
    """
    
    best_score = 0
    best_r0, best_r1 = None, None
    
    for r0 in range(100e3, 10e6, step=200e3):  # Wire resistance
        for r1 in range(50e3, 500e3, step=50e3):   # Trace resistance
            
            # Simulate RC circuit
            # For RC: V(t) = V_final × (1 - e^(-t/RC))
            
            tau_fast = R0 × C  # Time constant for fastest touch
            tau_slow = (R0 + (n_nodes-1)×R1) × C
            
            t_fast = tau_fast × ln(V_final / (V_final - 2.5))
            t_slow = tau_slow × ln(V_final / (V_final - 2.5))
            
            time_separation = abs(t_slow - t_fast)
            
            # Check constraints
            if all_constraints_satisfied:
                if time_separation > best_score:
                    best_score = time_separation
                    best_r0, best_r1 = r0, r1
    
    return best_r0, best_r1
```

This optimization is done **symbolically** using lcapy + sympy to find exact time values.

---

**End of Presentation Document**
