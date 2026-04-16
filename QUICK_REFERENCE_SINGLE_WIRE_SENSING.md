# Single-Wire Sensing: Quick Reference & Key Differences

## One-Minute Summary

**What:** Single conductive wire running through a 3D-printed object detects touches at multiple points  
**How:** Different locations have different electrical resistance → different voltage response times → detect which node was touched  
**Why One Wire:** Optimal R0 (main wire) + R1 (trace resistance) creates unique time signatures per location  

---

## v30 vs v3.5 at a Glance

```
┌─────────────────────────────────────────────────────────┐
│                         v30 vs v3.5                     │
├─────────────────────────────────────────────────────────┤
│                                                         │
│ ROUTING SPACE:                                          │
│                                                         │
│ v30: Contour layers (surface-based)                     │
│      ●─ Nested surfaces guide paths ─●                 │
│      Follows "highways" = contours                      │
│                                                         │
│ v3.5: Voxel grid (space-based)                          │
│      ●─ All interior cells are valid ─●                │
│      A* explores full 3D space                          │
│                                                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│ PATHFINDING:                                            │
│                                                         │
│ v30: Walk along contours between nodes                  │
│      Simple, deterministic                              │
│                                                         │
│ v3.5: A* + Detours + Serpentine + Waypoints            │
│      Sophisticated, multiple strategies                 │
│                                                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│ LENGTH CONTROL:                                         │
│                                                         │
│ v30: Fixed paths → optimize R0, R1 to match             │
│      Resistance tuning is primary control               │
│                                                         │
│ v3.5: Target lengths → adapt paths to fit               │
│      Geometric spiral/serpentine is primary control     │
│                                                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│ MULTI-SEGMENT SPACING:                                  │
│                                                         │
│ v30: Implicit (contour layers separate by default)      │
│      May intersect at junctions                         │
│                                                         │
│ v3.5: Explicit (blocked_radius enforcement)            │
│      Guaranteed minimum distance between routes         │
│                                                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│ ROBUSTNESS:                                             │
│                                                         │
│ v30: Works well for convex, simple geometries           │
│      May fail on complex topologies                     │
│                                                         │
│ v3.5: Works for any closed geometry                     │
│      Multiple fallback strategies                       │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## The Circuit Explanation (Simplified)

### Three Components That Matter

```
   Battery (5V)
       │
       │ R0 (Main Wire Resistance)
       ├─────>
       │
    Node 1 ──C── Sensor
       │
       │ R1 (Trace Resistance)
       ├─────>
       │
    Node 2 ──C── Sensor
       │
       │ R1 (Trace Resistance)
       ├─────>
       │
    Node 3
       │
      ...etc...
```

### Why Different Touches Have Different Times

```
TOUCH AT NODE 1 (closest to battery):
  Total Resistance = R0
  Time to 2.5V = Very fast (small RC time)
  
TOUCH AT NODE 3 (middle):
  Total Resistance = R0 + 2×R1  
  Time to 2.5V = Medium (medium RC time)
  
TOUCH AT NODE 5 (far from battery):
  Total Resistance = R0 + 4×R1
  Time to 2.5V = Slow (large RC time)

┌────────────────────────────────┐
│ Timer Readings (example):       │
│ Node 1 detected at: 0.3ms      │
│ Node 3 detected at: 0.6ms      │
│ Node 5 detected at: 0.9ms      │
│                                │
│ Each has UNIQUE time →         │
│ Identify which was touched!    │
└────────────────────────────────┘
```

---

## How Each Version Works

### v30 Workflow (Research Paper)

```
Step 1: Create Contours
  Mesh → Slice into nested layers (like onion rings)
  
Step 2: Route Along Contours  
  Node A → Follow contours → Node B
  (paths predetermined by surface structure)
  
Step 3: Calculate Resulting Resistances
  Path length + material property → R_actual
  
Step 4: Optimize R0 and R1
  "What R0 and R1 values work best with these paths?"
  Try many combinations, find best time discrimination
  
Step 5: Print
  Geometry ready to print
  
KEY INSIGHT: Paths are FIXED, resistors are TUNED
```

### v3.5 Workflow (Modern Implementation)

```  
Step 1: Build 3D Voxel Grid
  Create uniform 3D grid, mark valid cells (inside mesh)
  Remove cells too close to boundary
  
Step 2: Create Roominess Map
  For each cell: count neighbors
  Identify tight spots vs open spaces
  
Step 3: Route First Segment
  Node A → Node B using A* pathfinding
  Can choose: direct, serpentine, or waypoint-based
  Target: hit desired length (e.g., 2000mm)
  
Step 4: Mark Segment as Blocked
  Block all cells within `blocked_radius` 
  Next segment must route around it
  Result: Non-intersecting parallel paths
  
Step 5: Route Remaining Segments
  Repeat Steps 3-4 for all node pairs
  Each respects previous blocking zones
  
Step 6: Generate 3D Geometry
  Connect path waypoints with cylinders
  Add spheres at junctions
  Union into solid object
  
Step 7: Print
  Geometry ready to print
  
KEY INSIGHT: Resistances emerge from geometry, 
             but geometry is CONTROLLED by targets
```

---

## Code Structure Comparison

### v30 Conceptual Code

```python
# Load model
mesh = load_mesh("bunny.stl")
touch_nodes = [node1, node2, node3, ...]

# Generate contours
contours = generate_contours(mesh, num_layers=15)

# Route using contours
paths = []
for start, end in consecutive(touch_nodes):
    path = contour_based_route(start, end, contours)
    paths.append(path)

# Optimize resistances to fit curved paths
r0, r1, score = optimize_resistances(
    n_nodes=len(touch_nodes),
    wire_candidates=(100k, 10M, step=200k),
    trace_candidates=(50k, 500k, step=50k)
)

# Generate geometry
geometry = create_geometry(paths, r0, r1)
```

### v3.5 Conceptual Code

```python
# Load model
mesh = load_mesh("bunny.stl")
touch_nodes = [node1, node2, node3, ...]

# Build 3D grid with clearances
valid_cells = build_grid(
    mesh=mesh,
    wire_diameter=0.5mm,
    clearance=0.5mm
)

# Build roominess map (optimization hint)
roominess = map_roominess(valid_cells)

# Route with target lengths
target_lengths = [2000, 1500, 1800]  # mm per segment

paths = route_with_spacing(
    nodes=touch_nodes,
    valid_cells=valid_cells,
    target_lengths=target_lengths,
    blocked_radius=3  # cells
)

# Generate geometry
geometry = create_geometry(paths)
```

---

## Why v3.5 is Better (and Worse)

### v3.5 Advantages ✅

| Advantage | Reason |
|-----------|--------|
| **Handles complex geometry** | A* can route around interior obstacles |
| **Guaranteed spacing** | `blocked_radius` enforces minimum distance |
| **Predictable lengths** | Serpentine fill + waypoint detours hit targets |
| **Non-intersecting routes** | Explicit blocking prevents collisions |
| **Robust error handling** | Multiple fallback strategies |
| **Better for manufacturing** | Production-grade geometry control |

### v30 Advantages ✅

| Advantage | Reason |
|-----------|--------|
| **Conceptually simple** | Contours are intuitive |
| **Faster to compute** | Fewer path options to try |
| **Good for research** | Clear validation of sensing principle |
| **Surface-aware** | Respects mesh topology naturally |

### Making the Choice

**Use v30 if you're:**
- Studying the research paper
- Working with simple convex objects
- Validating theory

**Use v3.5 if you're:**
- Manufacturing real products
- Dealing with complex geometry
- Need guaranteed spacing
- Want reliable results

---

## Key Parameters & What They Mean

### v3.5 Parameters

| Parameter | What It Is | Impact |
|-----------|-----------|--------|
| **wire_diameter_mm** | Physical wire thickness | Larger = more space needed, more resistance |
| **blocked_radius** | Cells to block per segment | Larger = more spacing, harder to route |
| **target_length** | Goal path length per segment | Used to trigger serpentine/detours |
| **roominess_threshold** | Tight vs open space cutoff | Guides corridor sizing |

### v30 Parameters

| Parameter | What It Is | Impact |
|-----------|-----------|--------|
| **r0** | Main wire resistance | Affects RC time constant |
| **r1** | Trace segment resistance | Affects resistance between nodes |
| **num_contours** | How many nested layers | More = finer routing options |
| **voltage_threshold** | Detection trigger level | Usually 2.5V (sensor spec) |

---

## The Math Behind It All

### Why Touch Detection Works

```
RC Circuit Response:
  V(t) = V_final × (1 - e^(-t/τ))
  
  where τ = RC (RC time constant)
  
For touch at different locations:
  Node 1: τ_1 = R_0 × C
  Node 2: τ_2 = (R_0 + R_1) × C
  Node 3: τ_3 = (R_0 + 2×R_1) × C
  
Time to reach 2.5V threshold:
  t = τ × ln(V_final / (V_final - 2.5))
  
If R increases → τ increases → time increases
→ Different locations reach threshold at different times!
```

### Optimization Goal

```
Maximize: (t_slowest - t_fastest)
Subject to:
  - t_slowest < 1ms (timeout)
  - All times positive and realistic
  - r0 and r1 in feasible ranges

Good optimization score = times are well-separated
→ Easy to distinguish which node was touched
```

---

## Common Questions Answered

**Q: Why can one wire detect 5 touch points?**  
A: Each point is at a different distance from the battery along the wire. Each distance has a unique sum of resistances, creating a unique RC time constant. Touch at Node 1 vs Node 5 produces very different voltage rise times.

**Q: What happens if I touch two nodes at once?**  
A: You get a complicated response that's hard to interpret. Most systems assume single touches.

**Q: Why use resistive traces and not just vary wire diameter?**  
A: Because thin wires are fragile. v3.5 approach uses a thick strong wire (R0) and separate thin resistive traces (R1) connecting to touch nodes. More robust.

**Q: Can this work wirelessly?**  
A: Not without significant modification. The capacitive coupling to the sensor requires a physical connection through the wire network.

**Q: What's the maximum number of nodes?**  
A: Theoretically many, but practically limited by:
  - Resistance getting too high (signal loss)
  - Time scales becoming too compressed (hard to distinguish)
  - Manufacturing complexity
  
Typical: 4-10 nodes per wire network

---

## From Research to Reality

### v30 (Paper)
- **Goal:** Prove the concept works
- **Result:** Yes! Single wire + timing = position detection
- **Publication:** Research paper validated

### v3.5 (Product)  
- **Goal:** Make it practical for manufacturing
- **Result:** Robust implementation in Rhino 8 plugin
- **Status:** Ready for complex geometries

### Next Steps
- Better spacing enforcement
- Automated parameter optimization
- Multi-wire networks for larger objects
- Integration with sensing electronics

---

## For Your Professor

### Key Talking Points

1. **The Insight:**  
   "By measuring RC time constants along a single wire, you can identify WHICH location was touched based on WHEN the voltage reached threshold."

2. **The Progress:**  
   "v30 proved it works. v3.5 made it practical by using a robust 3D grid-based routing algorithm instead of surface contours."

3. **The Trade-off:**  
   "v30 is conceptually cleaner. v3.5 is more robust but more complex. Both valid for their purposes."

4. **The Technical Achievement:**  
   "Multi-segment spacing enforcement (blocked_radius) was the key innovation enabling v3.5's reliability."

5. **The Challenge:**  
   "Simultaneously satisfying: target lengths, minimum spacing, printability, and electrical properties."

---

**Good luck with your presentation! 🚀**
