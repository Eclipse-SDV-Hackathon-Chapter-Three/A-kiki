# Police Chase System: Path Planning & Control

## 1. Shortest Path Generation

### GRP (GlobalRoutePlanner) Based Routing

**Core Technology: CARLA GlobalRoutePlanner with A\* Algorithm**

* Utilizes **CARLA's GlobalRoutePlanner (GRP)** with internal **A\* algorithm**
* Computes **shortest path through road network graph** from origin to destination

**A\* Algorithm Intuition**

* **Dijkstra**: Only considers **distance traveled so far (g)**
* **A\***: Adds **estimated remaining distance (h)**
   * `f = g + h` (actual distance + heuristic estimate)
* Result: **Much faster and more realistic pathfinding**
* **Same principle used in real-world navigation systems**

**Key Features**

* **Excellent for real-time search** (fast performance)
* **Optimal route guarantee based on road network**

---

### Real-Time Adaptive Routing for Dynamic Targets

**Challenge**

Our system requires continuous path recalculation as police vehicles track criminals whose GPS positions change in real-time. This demands both:
* **Computational efficiency** for rapid updates
* **Optimal route generation** that adapts to moving targets

**Solution: Area-Based Multi-Point Routing**

**Objective**

* Beyond simple "follow current lane" logic
* Considers **all candidate points within origin/destination areas**
* Benefits:
   * **Not restricted to forward-direction lanes only**
   * **Enables U-turns, reverse driving, and true shortest-distance paths**

**Emergency Mode: Lock-On Signal**

When a criminal vehicle **Lock-On signal** is received, the system enters emergency mode:
* ⚠️ **Traffic signal waiting**: Suspended
* ⚠️ **Reverse driving**: Permitted
* ⚠️ **Illegal U-turns**: Allowed

This ensures **maximum distance coverage** in critical pursuit situations.

**Process**

1. Define circular search areas around origin and destination
2. Generate 6-20 candidate points within each area
3. Execute GRP (A*) for all candidate point combinations
4. Select the shortest final route **in real-time**

**Example**

* 10 origin candidates × 10 destination candidates = **100 route candidates**
* **Rapid computation** selects optimal path among all possibilities
* **Automatic selection of shortest distance path** including non-conventional maneuvers

**Key Takeaway**

> "By combining **GRP's fast A\* search** with **multi-candidate area exploration**, our police vehicles can find **realistic shortest paths (including U-turns and reverse driving)** that adapt to real-time changes in criminal vehicle positions."

---

## 2. Controller Module

### System Overview

**Our System = 3 Independent Controllers Working Together**

* **PID Controller**: Speed control (throttle/brake) + Soft Start specialization
* **Stanley Controller**: Steering control (lateral) + Heading/CTE correction
* **VehicleUnstuck Controller**: Emergency auto-recovery (2-phase escape)

**→ Result: Precise real-time control at 50ms intervals for perfect police pursuit!**

---

### Why Three Controllers?

In autonomous driving simulations, **computational delays** can cause vehicles to miss critical steering timing, resulting in:
* Getting stuck at intersections
* Collision with obstacles
* Failed maneuvers

Our **triple-controller architecture** handles these real-world challenges:
* **PID**: Handles speed smoothly
* **Stanley**: Provides precise steering
* **VehicleUnstuck**: Automatically recovers from stuck situations

---

### PID Controller

**Speed Control**

* **P (Proportional)**: Current error → Immediate response
* **I (Integral)**: Accumulated error → Long-term bias correction
* **D (Derivative)**: Rate of change → Overshoot/oscillation suppression
* **Soft Start**: Gradual acceleration when chase initiates

**Result**: Smooth and stable speed control

---

### Stanley Controller

**Steering Control**

* **Heading Error**: Vehicle heading vs. path heading difference
   * Measures "how much the vehicle's orientation deviates from the road"
* **Cross-Track Error (CTE)**: Vehicle position vs. path perpendicular distance
   * Measures "how far the vehicle has drifted laterally from road center"
* Computes steering angle by simultaneously considering both errors
* **Lookahead distance (5m)** + Enhanced steering response

**Result**: 2× more precise path tracking than PID alone

---

### VehicleUnstuck Controller

**Automatic Stuck Situation Recovery**

**Problem**: During simulation, slow computation can cause the vehicle to miss steering timing and get stuck

**Solution**: Automatic detection and recovery

* **Detection**: `throttle > 0.8 && speed < 0.5` for 2+ seconds
* **Phase 1**: Reverse + steering
* **Phase 2**: Forward + counter-steering

**Result**: Automatic recovery without driver intervention, ensuring continuous pursuit

---

### Integrated Control System

**System Architecture**

* **Control cycle**: 50ms
* Independent controllers cooperating with situational priority
* **Final output**: VehicleControl (steer, throttle, brake)

**Performance**

* Real-time cooperative pursuit with multiple vehicles
* Robust handling of complex urban scenarios
* Automatic recovery from computational delays

---

## Key Achievements

✅ **Real-Time Adaptive Routing**: Dynamic path recalculation for moving criminal targets  
✅ **Emergency Mode**: Traffic rule override during Lock-On situations  
✅ **Intelligent Path Planning**: A* with area-based multi-point routing (100+ candidates)  
✅ **Precise Control**: Triple-controller architecture (PID + Stanley + Unstuck)  
✅ **Real-time Performance**: 50ms control loop for responsive pursuit  
✅ **Robust Recovery**: Automatic handling of stuck situations from timing delays  
✅ **Scalable**: Supports multi-vehicle cooperative operations
