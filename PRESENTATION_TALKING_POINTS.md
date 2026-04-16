# Professor Presentation: Talking Points & Script

**Duration:** 15-20 minutes (adjust based on your needs)
**Audience:** Professors from various backgrounds (not all will be EE-focused)
**Goal:** Demonstrate deep understanding of both v30 and v3.5 implementations

---

## Opening (1 minute)

### Hook
"Imagine this: you want to detect where someone touches a 3D-printed object. Normally, you'd need separate wires or sensors for each touch point. But what if you could use **just ONE wire** and figure out WHERE it was touched just by measuring **WHEN** the voltage reaches a threshold?"

### What You'll Cover
"Today I'm walking you through exactly how that works. We'll look at two implementations: first the original research version (v30), then the production-ready version (v3.5), and I'll explain both the similarities and key differences."

---

## Part 1: The Big Picture (2-3 minutes)

### Start with the Hardware
"So the basic setup is simple:
- One continuous conductive wire running through a 3D-printed object
- That wire connects touch nodes on the surface
- A battery on one end, a sensor on the other
- Total: one wire, multiple detectable touch locations"

### The Magic Principle
"The genius insight is this: **resistance changes along the wire because each touch node is at a different distance from the battery.**

Think of it like this..." [Reference Diagram 1]

"The closer a touch point is to the battery:
- Lower total resistance
- Faster voltage rise time
- Sensor detects it quickly (maybe 0.3 milliseconds)

Touch a point far from the battery:
- Must go through multiple resistive segments
- Slower voltage rise time  
- Takes longer to reach threshold (maybe 0.9 milliseconds)

Different locations = different times = you can identify WHICH point was touched."

### The Circuit Behavior
"The circuit acts like an RC network. When you apply voltage, the capacitance at the touch node determines how fast the voltage rises. The time to reach threshold depends on the total series resistance.

Mathematically:
- V(t) = V_final × (1 - e^(-t/RC))
- Time to reach 2.5V = R × C × ln(ratio)

The key is: **different R values = different times**"

---

## Part 2: The Original Approach - v30 (3-4 minutes)

### The Research Context
"The researchers doing this project published a paper validating this concept. The v30 version I'm looking at is the original implementation tool. It's elegant and conceptually clean."

### How v30 Works - Step 1: Contours
[Reference Diagram 2]

"First, they take the 3D mesh and slice it into nested contour layers—like an onion. Each layer is a nested surface inside the object.

Why contours? Because they're natural guides. They preserve the object's topology and create a structure that's easy to reason about."

### Step 2: Routing Along Contours
"Next, they route the wire by walking along these contours. Starting at one touch node, they trace a path on the inner contours, then jump to the next touch node on a different contour level.

The algorithm is straightforward:
- Find closest safe point on adjacent contour
- Move to that point
- Repeat until reaching the destination

It's deterministic and relatively fast to compute."

### Step 3: Calculating Resistances
"Once the paths are designed, they can calculate:
- Path length = 2000mm
- Material properties (copper or conductive filament)
- Result: Actual resistance value

So now they know: 'This path will have approximately 150 kΩ of resistance'

But—and here's the problem—the paths are **fixed**. They can't change where the wire goes after the contours are generated."

### Step 4: Optimization - The Clever Part
"This is where the research contribution comes in. They ask: **'Given these fixed paths with known resistances, what values of R0 (the main wire) and R1 (the trace resistance between nodes) maximize the time discrimination?'**

They solve this using symbolic circuit analysis:
1. Build a symbolic circuit model (using lcapy library)
2. Simulate voltage transients for every possible R0, R1 combination
3. For each combination, find the time when voltage reaches 2.5V
4. Calculate: time_slow - time_fast (the 'score')
5. Pick the (R0, R1) pair with the highest score

This brute-force grid search explores millions of combinations and finds the optimal values."

### Why This Works
"The insight is: if you have WELL-SEPARATED time values between different nodes, errors are unlikely. If Node 1 responds at 0.3ms and Node 4 at 0.95ms, you can confidently say '0.4ms measured = Node 1 was touched.'

v30 ensures this separation by tuning the circuit values."

### Advantages of v30
"✓ Conceptually elegant—the contour structure guides everything  
✓ Fast to compute—limited path choices  
✓ Good for research—validates the sensing principle  
✓ Clear optimization target—maximize time discrimination"

### Limitations of v30
"✗ Paths are fixed by surface topology  
✗ Doesn't handle complex interiors well  
✗ Contours may not exist in some regions  
✗ Can't guarantee spacing between routes  
✗ No control over path lengths directly"

---

## Part 3: The Modern Approach - v3.5 (4-5 minutes)

### The Problem Statement
"Fast forward to v3.5. The researchers now want to make this practical for manufacturing. They're working in Rhino, and they have new requirements:

- Work with ANY closed geometry, no matter how complex
- Guarantee non-intersecting paths (for manufacturing reliability)
- Control path lengths directly (not just by resistance tuning)
- Handle tight internal spaces robustly

Those requirements led to a completely different architecture."

### The Key Innovation: Voxel Grid + A*
[Reference Diagram 3]

"Instead of surface contours, v3.5 uses a **3D voxel grid**. Think of the interior space as millions of tiny cubic cells. Each cell is either:
- Valid (inside the mesh with clearance)
- Blocked (too close to boundary, or occupied by previous routing)

Now they can use A* pathfinding—a classic algorithm from robotics—to find optimal routes through this grid."

### Step 1: Build the Grid
"The first step is meticulously creating the valid grid:

For every possible point in 3D space:
1. Is it inside the mesh? 
2. Is it far enough from the mesh boundary (safety clearance)?
3. Is it not blocked by previous routing?

If ALL three are yes, that cell is valid. Otherwise, it's blocked.

Result: a set of ~500,000 valid cells where the wire can safely route."

### Step 2: Roominess Map
"This is clever. For each valid cell, they count how many other valid cells are nearby (in a 3×3×3 neighborhood). This number is the 'roominess.'  

Cells in wide-open spaces get high roominess scores. Cells in tight bottlenecks get low scores.

Why compute this? Because it tells them where there is room to spiral and detour. This hint guides later routing strategies."

### Step 3: Smart Routing with Multiple Strategies
"Here's where v3.5 gets its robustness. Instead of one routing method, they try multiple strategies in sequence:

**Strategy 1 - Direct A* Path:**
- Use classic A* algorithm  
- Find shortest valid path
- Cost = distance traveled
- Result: direct route if possible

But what if the direct path is too SHORT for the target length?

**Strategy 2 - Serpentine Fill:**
[Reference Diagram 4]

If direct is too short, they fill the interior space by spiraling back and forth through layers:
1. Build a corridor around the direct path
2. Identify sweep direction (longest axis)
3. Snake back-and-forth through corridor layers
4. Adjust spacing based on roominess

This can extend the path by 2-3x!

**Strategy 3 - Waypoint Detours:**
If spiral still isn't enough, they find intermediate waypoints (high-roominess cells) and route through them:
- Start → Waypoint 1 → Waypoint 2 → Target
- Greedily expand toward target length
- This is more intelligent than random spiraling

They try these three in sequence until one achieves the target length."

### Step 4: Multi-Segment Spacing - The Key Innovation
[Reference Diagram 5]

"Here's what makes v3.5 production-ready: **guaranteed spacing between different route segments.**

After routing the first segment (Node 1 → Node 2):
1. They mark all cells in that path
2. They expand that marking by 'blocked_radius' (maybe 3 cells)
3. Those blocked cells are now forbidden for the next route

When routing Node 2 → Node 3, the A* pathfinding **must avoid all blocked cells**. There's an exception: a small zone around the next node lets them connect, but otherwise they're forced away.

Result: every segment is separated from previous ones by a guaranteed minimum distance!

This is CRITICAL for manufacturing—you don't want parallel wires touching."

### Step 5: Generate Geometry
"Once all paths are determined, they create 3D geometry:
- Connect consecutive waypoints with cylinders (the conductive wire)
- Add spheres at junctions (smooth transitions)
- Union everything into one solid

This becomes the STL file they send to the 3D printer."

### Why v3.5 Is Better for Manufacturing
"✓ Handles ANY topology (complex shapes, internal obstacles)  
✓ Non-intersecting paths are GUARANTEED (not assumed)  
✓ Length control is DIRECT (spirals and detours hit targets)  
✓ Multiple routing strategies (robust error handling)  
✓ Clear geometric constraints (safe for production)"

### Trade-offs of v3.5
"✗ More complex code (multiple algorithms)  
✗ Slower computation (more path options explored)  
✗ Less conceptually obvious than contours  
✗ Requires voxel grid generation (memory/compute overhead)"

---

## Part 4: Side-by-Side Comparison (2 minutes)

[Reference Diagram 6 and Quick Reference]

### The Fundamental Difference
"The core difference is **what guides the routing:**

**v30:** Surface topology guides routing. Contours determine where the wire goes.

**v3.5:** Target specifications guide routing. 'Route 2000mm from A to B'—the algorithm finds how."

### Path Philosophy
"v30: 'Given these paths, find the circuit values that work'  
v3.5: 'Given these targets, find the paths that work'

These are almost opposite mindsets!"

### The Resistance Question
"Here's an interesting difference:

**v30:** Optimizes R0 and R1 by tuning component values (external resistors added to circuit if needed)

**v3.5:** R0 and R1 emerge naturally from the path geometry. The microcontroller just measures what it gets. (No external components needed!)

This makes v3.5 simpler to manufacture."

### When to Use Each
"**v30 is great if:**
- You're validating the research concept  
- You're working with simple convex shapes  
- You want a straightforward, intuitive implementation
- Speed of computation matters

**v3.5 is great if:**
- You're manufacturing real products  
- You need robust support for complex geometry
- You want guaranteed spacing between routes  
- You need precise control over electrical properties"

---

## Part 5: The Technical Highlights (2-3 minutes)

### Circuit Analysis Deep Dive
"Let me dig into the math briefly. The circuit is an RC network:

When you touch Node 1 (closest to battery):
- Total R = R0 (main wire only)  
- Small RC time constant
- Fast voltage rise

Touch Node N (farthest):
- Total R = R0 + (N-1) × R1
- Large RC time constant  
- Slow voltage rise

The time to reach 2.5V threshold:
  t = RC × ln(V_supply / (V_supply - V_threshold))

So if R doubles, the time roughly doubles. This is exploited for discrimination."

### Why Optimization Matters
"Getting R0 and R1 right is crucial. If they're too close to each other:
- Times are compressed (hard to distinguish)
- Errors increase

If they're too large:
- Voltage never reaches 2.5V (timeout exceeded)  
- Or rises too slowly (outside usable timing window)

The optimization finds the Goldilocks spot: well-separated response times within the valid timing window.

v30 does this mathematically. v3.5 relies on the designer to choose appropriate path lengths."

### The Spacing Mathematics
"In v3.5, blocked_radius is the key parameter:

blocked_radius = ceil((wire_diameter + path_separation) / grid_step) + 1

Example:
- Wire diameter = 0.5mm
- Path separation = 0.5mm (safe gap)
- Grid step = 0.5mm
- blocked_radius = ceil(1.0 / 0.5) + 1 = 3 cells

This ensures at least 0.5mm clearance between adjacent routes."

---

## Part 6: Visual Walkthrough (2 minutes)

### Using the Diagrams
[Show Diagram 1 - Basic Principle]
"Remember, this is the core idea. One wire carries the signal, different touch locations give different times."

[Show Diagram 3 - Voxel Grid]
"In v3.5, this grid represents all possible routing locations. The algorithm explores millions of possibilities to find the best path."

[Show Diagram 5 - Blocking]
"This blocking mechanism is what ensures multi-segment isolation. It's the key innovation that made v3.5 production-ready."

[Show Diagram 8 - Touch Detection]
"And finally, here's what the sensor sees. Each node creates a unique voltage response time—the fingerprint that identifies the touch location."

---

## Part 7: Closing (1-2 minutes)

### The Journey
"So in summary:

**v30** proved the concept works. A single wire with optimized resistances CAN detect multiple touch points using RC timing discrimination.

**v3.5** solved the engineering problem. How do you route that single wire through arbitrary complex geometries while maintaining reliability and control?

The progression is: research → validation → practical implementation."

### Key Takeaways
1. **One wire + timing = position detection** (the research insight)
2. **Contours work for simple shapes** (v30)
3. **Voxel grids work for anything** (v3.5)
4. **Spacing enforcement is critical** (manufacturing reality)
5. **Multiple strategies = robustness** (engineering practice)

### Why This Matters
"Beyond the research novelty, this represents a real engineering challenge:

- How do you take theoretical sensing concept
- And make it practical for 3D printing
- With guaranteed manufacturing quality
- And robust behavior in the real world

That's what v3.5 solves."

### Questions
"I'm happy to dig deeper into any aspect:
- The circuit math and optimization
- The pathfinding algorithms  
- The geometry generation
- The practical manufacturing challenges
- How this compares to other sensing approaches"

---

## Q&A Prep - Common Questions

### Q: Why can't you just use multiple wires?
**A:** "You could, but then you need either:
- Separate connections for each wire (complex harness)
- Multiplexing electronics (more components)

Single wire + timing is elegant because you need only one electrical connection."

### Q: What if someone touches between two nodes?
**A:** "They'd get an intermediate response time. The system might interpolate or report 'between Node 2 and Node 3.' Or you could add more nodes for finer resolution—it's a trade-off between manufacturing complexity and sensing resolution."

### Q: How fast does the detection work?
**A:** "The voltage rise is on millisecond timescales (1-10ms typical). So detection latency is <10ms. That's fast enough for practical interactions like button presses."

### Q: Does this work on non-conductive objects?
**A:** "Yes! The wire itself is conductive (copper or conductive filament). The object could be plastic, resin, or anything else. The capacitive coupling to the touch point is what matters."

### Q: What about noise/interference?
**A:** "That's a challenge in real systems. You'd need:
- Shielding around the wire  
- Filtered power supply
- Software debouncing
- Calibration routine

v3.5 handles the routing; the sensing electronics handle noise."

### Q: Could you use this for pressure sensing?
**A:** "Interesting question! The RC time constant is primarily dependent on the resistance path, not much on pressure. However, if pressure changed the contact resistance at the touch point, you could theoretically extract that. But that's more complex than this system was designed for."

### Q: How does v3.5 handle dead-ends or impossible routing?
**A:** "Good question. If A* can't find a path, it fails over to serpentine fill. If that doesn't give enough length, it tries waypoint detours. If all three fail, the algorithm returns an error. In practice, the voxel grid is usually large enough that some path exists. And if not, the user adjusts parameters (like blocked_radius) to open up more space."

---

## If You Have Extra Time

### Deep Dive: Roominess Algorithm
"One elegant part of v3.5 is the roominess map. It's a local density metric:

```
roominess[cell] = count of valid neighbors within radius R
```

This has nice properties:
- High roominess = open space (room to spiral)
- Low roominess = tight bottleneck
- Used to guide serpentine spacing (wider spacing in open areas, tighter in bottlenecks)

It's algorithmic intuition: 'This space is narrow, so keep paths close. This space is open, so you can spread out.'"

### Deep Dive: A* vs Dijkstra
"v3.5 uses A* pathfinding, which is like Dijkstra's algorithm but with a heuristic:

**Dijkstra:** Explores equally in all directions, guaranteed shortest path

**A*:** Exploits a heuristic (usually Euclidean distance to goal) to bias search toward the target, faster for single-path problems

In v3.5: heuristic = straight-line distance to next node

This makes it much faster than pure Dijkstra."

### Deep Dive: Why Serpentine?
"The serpentine strategy is inspired by 3D printer nozzle paths. Printers fill solid areas by sweeping back-and-forth in parallel lines. v3.5 reuses this idea:

- Choose longest axis as sweep direction
- Layer through shortest axis  
- Alternate row direction

Result: efficient dense packing that respects the 3D space naturally."

---

## Slide Deck Organization Suggestion

**Slide 1:** Title + single-line thesis: "One wire, multiple touch points, timing-based detection"

**Slide 2:** Problem statement (diagram-free, text only)

**Slide 3:** Basic principle (Diagram 1)

**Slide 4:** Circuit model + RC timing

**Slide 5:** v30 Approach - Contours (Diagram 2)

**Slide 6:** v30 Algorithm flow

**Slide 7:** v30 Optimization process

**Slide 8:** v3.5 Approach - Voxels (Diagram 3)

**Slide 9:** v3.5 Key Strategy comparison

**Slide 10:** Spacing innovation (Diagram 5)

**Slide 11:** Serpentine fill (Diagram 4)

**Slide 12:** v30 vs v3.5 table (Diagram 6)

**Slide 13:** Complete system pipeline (Diagram 7)

**Slide 14:** Touch detection in action (Diagram 8)

**Slide 15:** Comparison summary

**Slide 16:** Conclusions + questions

---

## Tips for Delivery

1. **Start with the hook** - The premise is fascinating, use that to grab attention
2. **Use the diagrams liberally** - They're much clearer than words
3. **Relate to familiar concepts** - RC circuits, game A* pathfinding, 3D printer raster patterns
4. **Don't get stuck in math** - Mention it exists, explain intuition, skip derivations
5. **Emphasize the contrast** - The "opposite mindsets" of v30 vs v3.5 is memorable
6. **Show confidence** - You clearly understand this deeply, let that show
7. **Be honest about limitations** - Both versions have pros/cons, acknowledging this shows wisdom

---

**Good luck! You've got this. 🎓**
