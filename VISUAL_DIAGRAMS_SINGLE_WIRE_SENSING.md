# Visual Diagrams: Single-Wire Sensing Explained

## Diagram 1: How Single-Wire Sensing Works (Basic Principle)

```
THE PHYSICAL SETUP:
═════════════════════════════════════════════════════════════════

                            3D Printed Object
                            (with internal wire)
                                  ╱╲
                                ╱    ╲
                              ╱        ╲
                            ╱ Node 1    ╲
                          ╱─────○─────────╲
                        ╱    (Button 1)    ╲
                     ╱                      ╲
                   ╱   Wire Network Path:    ╲
                  ╱    ●
                 │     ├─● (Node 2)
                 │     ├─● (Node 3)
                 │     ├─● (Node 4)
                 │     └─● (Node 5)
                 │
                 │  Connected to:
                 │  Battery → Sensor → Microcontroller
                 │


THE ELECTRICAL PRINCIPLE:
═════════════════════════════════════════════════════════════════

                        5V Battery
                            │
                        R0 = Main Wire
                            │
                         o─Node 1─o
                         │   C    │
                     R1  │        │ Sensor
                         │ 2.5V   │ (High Impedance)
                         │        │
                         o─Node 2─o
                             R1
                         
When you TOUCH at Node 1:
  Current path: Battery → R0 (quick) → Node 1 sensor
  Time to 2.5V = FAST (maybe 0.3ms)
  
When you TOUCH at Node 3 (far away):
  Current path: Battery → R0 → R1 → R1 → Node 3 sensor
  Time to 2.5V = SLOW (maybe 0.7ms)
  
Microcontroller measures: Which took longest? 
  → Knows which node you touched!


THE MAGIC:
═════════════════════════════════════════════════════════════════

Touch Response Time Graph:

        Voltage (V)
        │
       2.5├─────────────────────────→ Detection Threshold
        │ ╱
        │╱     Node 1 touches (fast)
       1│╱       ╱ 
        │      ╱ ╱ Node 3 touches (slow)
        │    ╱  ╱
        │   ╱  ╱
        │  ╱  ╱
        │ ╱  ╱
        └─────────────────────────→ time (ms)
        0    0.3    0.5    0.7    1.0
        
        ↑ Node 1 detected here (0.3ms)
        
              ↑ Node 3 detected here (0.7ms)
              
        Difference = 0.4ms (easily distinguishable!)
```

---

## Diagram 2: v30 Approach (Contour-Based)

```
CONCEPTUAL FLOW:
═════════════════════════════════════════════════════════════════

    MESH INPUT              CONTOUR GENERATION           ROUTING
    ────────────            ──────────────────          ───────

      Bunny                   Outer Surface
      Model                        ╱╲
                               ╱──────╲
                            ╱─ Contour 1 ─╲
                         ╱──────────────────╲
                     ╱─ Contour 2 (middle) ─╲
                  ╱────────────────────────────╲
              ╱─ Contour 3 (inner core) ─────╲
              Node arrangements →

                              Path 1: Follow
                              inner contours
                              
                              ● Node A
                              │ (Contour 3)
                              │
                              ○─────→ (Contour 2)
                              │
                              ●─────→ (Contour 1)
                              Node B


ALGORITHM (v30):
═════════════════════════════════════════════════════════════════

1. Generate Contours (slice mesh into nested layers)
   Bunny mesh → 15 nested contour layers
   
2. For each node pair:
   a. Find starting point on outermost contour
   b. Ray-cast inward to find safe path
   c. Walk through contours layer by layer
   d. Minimize path distance
   e. Result: One smooth path between nodes

3. Analyze paths:
   Length of path → expected resistance
   Material properties → actual resistance value
   
4. Optimize R0 and R1:
   "For these fixed paths, what R0 and R1 maximize
    the time difference between fastest and slowest
    touch responses?"
    
   Try combinations until optimal found

5. Done! Paths + resistances = complete design


CHARACTERISTICS:
═════════════════════════════════════════════════════════════════

✓ Intuitive: Surface layers guide paths
✓ Fast: Only two dimensions of optimization
✓ Clean: Deterministic path generation

✗ Inflexible: Paths determined by surface contours
✗ Risk: Contours may not exist in all regions
✗ Spacing: May touch at connection points
✗ Simple geometries only: Fails on complex topology
```

---

## Diagram 3: v3.5 Approach (Voxel Grid + A*)

```
CONCEPTUAL FLOW:
═════════════════════════════════════════════════════════════════

    MESH INPUT          VOXEL GRID CREATION        PATHFINDING
    ────────────        ───────────────────        ────────────

      Bunny            ┌─────────────────┐
      Model            │ ✓ ✓ ✓ ✓ ✓ ✓ ✓ │
                       │ ✓ ✓ ✓ ✓ ✓ ✓ ✓ │
                       │ ✓ ✓ ✗ ✗ ✓ ✓ ✓ │ (✓=valid, ✗=blocked)
                       │ ✓ ✓ ✓ ✓ ✓ ✓ ✓ │
                       │ ✓ ✓ ✓ ✓ ✓ ✓ ✓ │
                       └─────────────────┘
                       
                       Grid cell = decision point
                       All cells explored equally
                       
                                            Multiple strategies:
                                            ┌─→ Direct A*
                                            ├─→ Serpentine Fill
                                            └─→ Waypoint Detours


ALGORITHM (v3.5):
═════════════════════════════════════════════════════════════════

Step 1: Build Valid Grid
────────────────────────
  For each point in 3D space around mesh:
    - Inside mesh? ✓
    - Distance from boundary > clearance? ✓
    - Not blocked by previous routing? ✓
  
  Result: Set of ~500K valid cells


Step 2: Compute Roominess Map
──────────────────────────────
  For each cell, count neighbors in 3×3×3 box:
  
    Roominess = count of neighbors in valid_cells
    
  Purpose: Identify tight spots vs open spaces
  
    High roominess: ●●●●●↓ Plenty of space
    Low roominess:  ●○‌■●○ Tight bottleneck


Step 3: Route First Segment (Node 1 → Node 2)
───────────────────────────────────────────────
  Target length = 2000mm
  
  Try Strategy 1: Direct A*
    Direct path found = 1200mm (too short!)
    ✗ Strategy 1 fails
  
  Try Strategy 2: Serpentine Fill
    Add spirals around direct path
    Spiral length = 2100mm (success!)
    ✓ Strategy 2 succeeds → Use this path
  
  Mark cells as BLOCKED:
    All cells within blocked_radius = 3
    Next segment must route around


Step 4: Route Second Segment (Node 2 → Node 3)
────────────────────────────────────────────────
  Blocked cells: [previous routing marked zone]
  Target length = 1800mm
  
  Try Direct A*:
    A* avoids blocked cells
    Found path = 1750mm (close enough!)
    ✓ Use this path
  
  Mark more cells as BLOCKED for next segment


Step 5: Repeat for All Remaining Segments
───────────────────────────────────────────
  Each segment respects previously blocked zones
  Result: Non-intersecting parallel paths!


Step 6: Generate 3D Geometry
────────────────────────────
  For each path waypoint sequence:
    - Connect consecutively with cylinders (wire radius)
    - Add spheres at junctions (smooth transitions)
    - Union all into single solid
  
  Result: 3D-printable geometry


CHARACTERISTICS:
═════════════════════════════════════════════════════════════════

✓ Robust: Handles any closed topology
✓ Spacing: Guaranteed minimum distance via blocked_radius
✓ Length control: Multiple strategies to hit targets
✓ Non-intersecting: Explicit enforcement
✓ Flexible: Can spiral, detour, adapt

✗ Complex: More code, more strategies
✗ Slower: More path options explored
✗ Less intuitive: 3D grid is abstract
```

---

## Diagram 4: Serpentine Fill Strategy (v3.5)

```
WHEN SERPENTINE IS NEEDED:
═════════════════════════════════════════════════════════════════

Direct A* path:           Target length:
   ╱╲                     Need: 2000mm
 ●─────●                  Have: 800mm
       │                  Shortfall: 1200mm
       
       Too short! ✗
       Need to expand to fill interior space


HOW SERPENTINE WORKS:
═════════════════════════════════════════════════════════════════

1. Identify routing corridor (box around direct path)

     ┌──────────────┐
     │              │
     │   ●────●     │  Corridor = box with margin
     │              │
     └──────────────┘

2. Choose sweep direction (longest axis):
   
     Vertical sweep        Horizontal rows
     (top to bottom):      (back and forth):
     
     ┌─────────────────┐  ┌─────────────────┐
     │ ●→→        →→● │  │ →→→→→→→→→→→ │
     ├─────────────────┤  ├─────────────────┤
     │ ←←●        ●←← │  │ ←←←←←←←←←← │
     ├─────────────────┤  ├─────────────────┤
     │ ●→→        →→● │  │ →→→→→→→→→→→ │
     └─────────────────┘  └─────────────────┘

3. Connect layers with vertical transitions:

     TOP LAYER:
     ●─→─→─→─→─→─●
               │
     BOTTOM:   │
     ●─←─←─←─←─←─●
        │
     Continue spiraling...

4. Result: Long serpentine path (2000mm+) ✓


SPACING CONSTRAINTS IN SERPENTINE:
═════════════════════════════════════════════════════════════════

Rows must be separated:
  spacing = self_avoid_radius + 2 cells
  
Example with radius=2:
  ┌──────────────────┐
  │ ●→→→→→→→→→●     │
  │             │    │
  │ (2 cell gap)│    │
  │             │    │
  │ ←←←←←←←←←●  │   │
  │  (connecting)    │
  │       ↓          │
  └──────────────────┘
  
Result: Non-overlapping snake pattern
         that fills interior efficiently
```

---

## Diagram 5: Blocking & Spacing (v3.5 Innovation)

```
THE MULTI-SEGMENT SPACING PROBLEM:
═════════════════════════════════════════════════════════════════

Without blocking (WRONG):
  
  Segment 1:  Node A ────●────● Node B
              
  Segment 2:  Node B ────●────● Node C
                           (overlaps!) ✗

With blocking (CORRECT):

  Segment 1:  Node A ────●────● Node B
              ●●●●●●●●●●●●●●●●●●  ← blocked zone
              ●●●●●●●●●●●●●●●●●●     (radius=3)
              
  Segment 2:           Node B
                          │
                          ↓ (finds route around block)
                      ┌──────────┐
                      │ ●      ● │ Node C
                      │ spiral  │
                      └──────────┘


HOW BLOCKING WORKS:
═════════════════════════════════════════════════════════════════

1. Route first segment (Node 1 → Node 2):
   
   (A*: explores valid cells, finds optimal path)
   
   Route 1: ●───●───●───●
            │   │   │   │
            Cells marked as routed


2. Expand blocked zone:
   
   blocked_radius = 3 cells
   
   Original route: ●───●───●───●
   
   Blocked zone:  ●●●●●●●●●●●●●
                  ●●●●●●●●●●●●●
                  ●●●●●●●●●●●●●
                  
   These cells are now forbidden for next route


3. Route second segment (Node 2 → Node 3):
   
   A* must find path that:
   - Starts at Node 2
   - Ends at Node 3
   - Avoids all blocked cells
   - Minimizes overlap with reservation zones
   
   Result: Route 2 forced away from Route 1 ✓


4. Exemption zones allow connections:
   
   blocked_exemption_radius small region around nodes
   
   Route 2 can pass through protected zone
   to actually connect to Node 2
   
   ●●●●●●●●●●●●●●
   ●●●●●┌──────●●   ← New route can enter here
   ●●●●●│●●●●●●●
   ●●Node2●●●●●●
   ●●●●●└──────●●
   ●●●●●●●●●●●●●●


RESULT:
═════════════════════════════════════════════════════════════════

Route 1: ═══════════════════
Route 2:                 ════════════════════
Route 3:                            ════════════

All parallel, non-intersecting, guaranteed spacing!
```

---

## Diagram 6: Comparison Side-by-Side

```
┌─────────────────────────────────────────────────────────────┐
│                     v30 vs v3.5 VISUAL                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  v30 (CONTOUR-BASED)          │  v3.5 (VOXEL + A*)          │
│                               │                              │
│  Space Model:                 │  Space Model:                │
│  ─────────────                │  ───────────                 │
│                               │                              │
│   Nested Contours             │   3D Voxel Grid              │
│   ╱╲╱╲╱╲╱╲                    │   ┌─────────────┐            │
│ ╱  ╱  ╱  ╱ ╲                  │   │ ✓ ✓ ✓ ✓ ✓ │            │
│ ╲ ╱ ╲╱  ╱ ╱                   │   │ ✓ ✓ ✓ ✓ ✓ │            │
│   ╲ Contours ╱                │   │ ✓ ✓ ✓ ✓ ✓ │            │
│                               │   │ ✓ ✓ ✓ ✓ ✓ │            │
│                               │   │ ✓ ✓ ✓ ✓ ✓ │            │
│                               │   └─────────────┘            │
│                               │                              │
│  Routing:                     │  Routing:                    │
│  ────────                     │  ──────────                  │
│  Walk contours                │  A* pathfinding              │
│  (on surfaces)                │  (in cells)                  │
│                               │                              │
│  ● ─→ ● ─→ ●                 │  ●═╗  ╔═●                   │
│  ↓   surface  ↓               │  ║ ║╔═╝ ║                   │
│  ● ─→ ● ─→ ●                 │  ║ ╚╣   ║                   │
│                               │  ╚═╤╨═══╝                   │
│                               │    •                        │
│                               │                              │
│  Length Control:              │  Length Control:             │
│  ────────────────             │  ────────────────            │
│  Paths fixed →                │  Paths adaptive →            │
│  Tune R0, R1                  │  Spiral/serpentine           │
│                               │                              │
│  Spacing:                     │  Spacing:                    │
│  ────────                     │  ─────────                   │
│  Implicit                     │  Explicit                    │
│  (contours naturally          │  (blocked_radius            │
│   separate)                   │   enforcement)              │
│                               │                              │
│  Issues:                      │  Advantages:                 │
│  ───────                      │  ────────────                │
│  • Simple shapes only         │  • Works on any shape        │
│  • May intersect              │  • Non-intersecting ✓        │
│  • Limited options            │  • Multiple strategies ✓     │
│                               │  • Guaranteed spacing ✓      │
│  Advantage:                   │  • Better for manufacturing  │
│  ──────────                   │                              │
│  • Conceptually clear         │                              │
│  • Fast computation           │                              │
│  • Good for research          │                              │
│                               │                              │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 7: The Complete System Architecture

```
COMPLETE v3.5 PIPELINE:
═════════════════════════════════════════════════════════════════

INPUT                          PROCESSING                      OUTPUT
─────                          ──────────                      ──────

STL Mesh                    ╔═══════════════╗              3D Geometry
  │                         ║  Build Valid  ║                  │
  ̲│                         ║  Grid         ║   (Ready to print)
   ̲                         ╚═══════════════╝
   │                              │
   │                              ↓
   │                         ╔═══════════════╗
   │                         ║ Roominess Map ║
   │                         ╚═══════════════╝
   │                              │
   │              ┌───────────────┼───────────────┐
   │              ↓               ↓               ↓
 Touch Nodes   ┌────────┐   ┌────────┐   ┌────────┐
   │           │   A*   │   │Serpent │   │Waypoint│
Touch Node     │  Path  │   │ Fill   │   │Detours │
Selection      └────────┘   └────────┘   └────────┘
   │              │               │               │
   │              └───────────────┼───────────────┘
   │                              │
   │                              ↓
   │                         ┌──────────────┐
   ├────────────────────────→│ Choose Best  │
   │                         │ Strategy     │
   │                         └──────────────┘
   │                              │
   │                              ↓
   │                         ┌──────────────┐
   │                         │  Mark Blocked│
   │                         │  Zones       │
   │                         └──────────────┘
   │                              │
   │           ┌──────────────────┴──────────────────┐
   │           ↓ Segment 1 done                      ↓
   │       Repeat for                          Combine all
   │       all segments                        paths into
   │           │                               one geometry
   │           └───────────────────────────────┘
   │                   │
   │                   ↓
   │              ╔═══════════════╗
   │              ║  Generate 3D  ║
   │              ║  Geometry     ║
   │              ║  (cylinders + ║
   │              ║   spheres)    ║
   │              ╚═══════════════╝
   │                   │
   │                   ↓
   └──────────────→ 3D Solid
                  (STL file)


RESISTANCE EMERGES FROM GEOMETRY:
═════════════════════════════════════════════════════════════════

Path length:
  Segment 1: 2000mm
  Segment 2: 1800mm  ─→  Total R1 = L × ρ / A
  Segment 3: 2200mm
  (Material: conductive filament, ρ, cross-section A)
  
Main wire:
  Wire length: depends on geometry  ─→  Total R0 = L × ρ / A
  
Result: R0 and R1 determined by the geometric design!
```

---

## Diagram 8: Touch Detection in Action

```
TIME-BASED POSITION DETECTION:
═════════════════════════════════════════════════════════════════

Physical Setup:
  
  Battery ─→ R0 (Main wire) ─→ ●─→ R1 ─→ ●─→ R1 ─→ ● ─→ ...
                            Node1    Node2   Node3
                            
Test Procedure:
─────────────────

  Microcontroller sends pulse to battery
  Measures time for voltage to reach 2.5V at sensor input
  
  
Scenario A: TOUCH AT NODE 1
─────────────────────────────

  Step 1: Pulse arrives at battery
          │
  Step 2: Voltage rises through R0
          ●R0 limited = ~3V (fast!)
          │
  Step 3: Reaches sensor at Node 1
          │
  ╔═════════════════════════════════╗
  ║ Time to 2.5V = 0.3ms ✓ DETECTED ║
  ║ Touch at Node 1!               ║
  ╚═════════════════════════════════╝


Scenario B: TOUCH AT NODE 2
────────────────────────────

  Step 1: Pulse arrives at battery
          │
  Step 2: Voltage rises through R0
          ●R0 limited = ~3V
          │
  Step 3: Must flow through R1 to reach Node 2
          ●R1 resistive = slower rise
          │
  Step 4: Reaches sensor at Node 2
          │
  ╔═════════════════════════════════╗
  ║ Time to 2.5V = 0.6ms ✓ DETECTED ║
  ║ Touch at Node 2!               ║
  ╚═════════════════════════════════╝


Scenario C: TOUCH AT NODE 4 (far)
──────────────────────────────────

  Step 1-2: Same as above
  Step 3: Must flow through R1 + R1 + R1 = 3×R1 !!
          ●Multiple R1 in series = slow rise
  Step 4: Reaches sensor at Node 4
          │
  ╔═════════════════════════════════╗
  ║ Time to 2.5V = 0.95ms DETECTED  ║
  ║ Touch at Node 4!               ║
  ╚═════════════════════════════════╝


TIMING DISCRIMINATION:
═════════════════════════════════════════════════════════════════

        Voltage vs Time
        
        2.5V ├─────────────────────────────
            │  Node 1           Node 2      Node 4
            │  (0.3ms)          (0.6ms)     (0.95ms)
        2.0 ├     ╱                ╱           ╱
            │    ╱                ╱           ╱
        1.5 ├   ╱                ╱           ╱
            │  ╱                ╱           ╱
        1.0 ├ ╱                ╱           ╱
            │╱                ╱           ╱
        0.5 ┼────────────────────────────
            │
        0.0 ├─────┬─────┬─────┬─────┬─────→
            0   0.3   0.6   0.9  1.2  time (ms)
                ▲     ▲     ▲
              N1?   N2?   N4?
              Det   Det   Det
              
        Each node produces UNIQUE time signature!
        Microcontroller compares measured time to table:
          Time = 0.3ms → Node 1 touched
          Time = 0.6ms → Node 2 touched
          Time = 0.95ms → Node 4 touched
```

---

## For Your Presentation Slides

### Slide 1: Title/Overview
Use Diagram 1 (How It Works)

### Slide 2: The Circuit
Use the electrical principle section of Diagram 1

### Slide 3: v30 Approach
Use Diagram 2 (Contour-Based)

### Slide 4: v3.5 Approach  
Use Diagram 3 (Voxel Grid + A*)

### Slide 5: Key Innovation - Spacing
Use Diagram 5 (Blocking & Spacing)

### Slide 6: Comparison
Use Diagram 6 (Side-by-side)

### Slide 7: System Architecture
Use Diagram 7 (Complete Pipeline)

### Slide 8: Live Detection Demo
Use Diagram 8 (Touch Detection)

---

**End of Visual Diagrams**
