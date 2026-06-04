# Cheese Pick-and-Place — System Architecture

**Author:** Rustam  
**Date:** June 2026  
**Version:** 1.0

---

## Table of Contents

1. [Problem Statement](#1-problem-statement)
2. [System Architecture Overview](#2-system-architecture-overview)
3. [Hardware Context](#3-hardware-context)
4. [Core Processing Pipeline](#4-core-processing-pipeline)
5. [Functional Components](#5-functional-components)
6. [System State Machine](#6-system-state-machine)
7. [Fault Handling](#7-fault-handling)
8. [Observability](#8-observability)
9. [Development and Deployment](#9-development-and-deployment)
10. [Time Estimate](#10-time-estimate)

---

## 1. Problem Statement

A conveyor belt carries cheese slices at constant speed, arriving at irregular intervals (4–6 seconds apart). A robot arm must pick each slice and place it into a specific grid cell of a moving container. The system must:

- Detect and track cheese slices while the belt is moving
- Compute feasibility before committing to a pick — the belt does not stop
- Place each piece into the correct grid slot
- Advance the container when a row is full
- Handle missed picks gracefully — the next slice continues normally
- Reconfigure between batches with different cheese dimensions

The system requires two types of calibration:

| Calibration | When | What |
|---|---|---|
| **Camera ↔ Robot** | Once at installation | Aligns camera coordinate frame with robot arm coordinate frame so detections can be converted to robot positions |
| **Batch dimensions** | Before each new cheese batch | Operator enters the new slice dimensions so the system adjusts grip depth, finger position, and descent height |

---

## 2. System Architecture Overview

The system is organized into three independent layers with clear boundaries.

```ini
┌─────────────────────────────────────────────────────────────┐
│                     OPERATIONAL LAYER                       │
│  Health Monitor · Fault Manager · Metrics · Foxglove        │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                     FUNCTIONAL LAYER                        │
│  Calibration · Vision · Tracking · Scheduling               │
│  Pick Planning · MoveIt2 · Robot Control                    │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                      PHYSICAL LAYER                         │
│  Orbbec Astra · Conveyor (constant speed) · xArm6 · Gripper  │
└─────────────────────────────────────────────────────────────┘
```

The Functional Layer performs the pick-and-place operation. The Operational Layer provides fault handling, diagnostics, and remote observability. The Physical Layer is the hardware interface.

---

## 3. Hardware Context

| Component | Specification |
|---|---|
| Robot arm | UFACTORY xArm6 (6-DOF, 5 kg payload) |
| Gripper | Mechanical parallel gripper with widened fingertips |
| Camera | Orbbec Astra series (structured light, RGB-D) |
| Conveyor | Flat belt, constant known speed, no encoder required |
| Server | Ubuntu Server (mini server, remote via VPN + SSH) |
| Visualization | Foxglove Studio via WebSocket |

### Camera Choice Rationale

The Orbbec Astra is a structured light camera that provides aligned RGB and depth frames. For this application it is sufficient because:

- Cheese slices are uniform within a batch — no classification needed
- Depth data enables 3D bounding box extraction directly without stereo reconstruction
- Structured light works well at short range (0.6–3.5 m), which covers a conveyor belt workspace

### Gripper Choice Rationale

A vacuum gripper would suit flat objects but cheese cut into slices has irregular side surfaces. The mechanical parallel gripper with widened fingertips grips the faces of the slice, which is reliable regardless of surface texture or moisture. Orientation-aware grasping is required (the gripper must align with the slice's long axis).

---

## 4. Core Processing Pipeline

```yaml
Orbbec Astra Camera
        │
        ▼
Vision Processing Node
  - RGB + Depth alignment
  - Point cloud generation
  - Plane removal (conveyor surface)
  - Euclidean cluster segmentation
  - 3D bounding box extraction
        │
        ▼
TF2 Transform Manager
  - Camera frame → Robot base frame
  - Calibrated at startup
        │
        ▼
Conveyor State Estimator
  - Takes detected object position + known belt speed
  - Predicts where the object will be at pick time
        │
        ▼
Object Queue
  - Decouples detection rate from execution rate
        │
        ▼
Task Scheduler
  - Feasibility check (reachable before exit?)
  - Assigns container slot
  - Fires pick/place command
        │
        ▼
Pick Planner
  - Computes pick pose (antipodal grasp, yaw-aligned)
  - Computes place pose (grid slot + container offset)
        │
        ▼
MoveIt2 Planner
  - Inverse kinematics
  - Collision-aware trajectory generation
  - Uses xArm6 URDF
        │
        ▼
Robot Controller
  - Executes joint trajectory
  - Controls gripper open/close
  - Returns execution feedback
```

---

## 5. Functional Components

### 5.1 Calibration Manager

There are two distinct types of calibration in this system. They must not be confused.

---

**Type 1 — Camera-to-Robot Calibration (once at installation)**

The Orbbec camera is mounted above the conveyor and sees the world in its own coordinate frame. The xArm6 robot operates in its own coordinate frame rooted at its base. These two frames are physically independent — the robot does not know where the camera is, and the camera does not know where the robot is.

This calibration computes the transform between the two:

```sh
P_robot = T_robot←camera × P_camera
```

Procedure: place a calibration target at several known positions, measure with both camera and robot, compute the rigid transform. This is done **once at installation** and stored as a static TF2 transform. After this, all camera detections are automatically expressed in robot coordinates.

---

**Type 2 — Batch Calibration (manual, before each product change)**

Guillaume explicitly stated: *"A calibration step between batches can therefore be assumed."* This means batch switching is a **manual operator action**, not an automatic process.

The system does not auto-detect batch changes from vision. The camera cannot reliably distinguish batch 1 cheese from batch 2 cheese, and attempting to do so would add unnecessary complexity. Instead:

1. The conveyor pauses between batches (factory-level event)
2. The operator enters the new cheese dimensions via a configuration interface
3. The system reconfigures and resumes production
4. From that point, every detected piece is assumed to match the loaded dimensions

This is not a sensor measurement. It is a deliberate configuration step. All pieces within a batch are identical so no per-piece measurement is ever needed during production.

Implemented as a **ROS2 Lifecycle Node**:

```sh
UNCONFIGURED → CONFIGURING → ACTIVE
                   ↑
            (on batch change)
             ACTIVE → CONFIGURING
```

Batch recipe example:

```yaml
cheese:
  length_mm: 100
  width_mm: 50
  height_mm: 20

gripper:
  finger_offset_mm: 5
  approach_height_mm: 80

container:
  rows: 2
  cols: 4
  slot_spacing_mm: 110
```

---

### 5.2 Vision Processing

**Design decision:** Generic geometric detection — no AI/ML. Guillaume confirmed this is simpler and sufficient for the current product scope.

Pipeline:

```
RGB + Depth frame
      │
      ▼
Point cloud generation (depth → 3D)
      │
      ▼
Statistical noise removal (PCL StatisticalOutlierRemoval)
      │
      ▼
RANSAC plane removal (conveyor surface)
      │
      ▼
Euclidean cluster segmentation (PCL)
      │
      ▼
Per-cluster: 3D bounding box + PCA yaw
      │
      ▼
Output: { x, y, z, yaw, length, width, height }
```

**Noise filtering:** Statistical outlier removal is applied before segmentation. Real structured-light cameras produce noisy depth readings due to sensor variance, surface reflectance, and ambient lighting changes. Without this step, a single noisy frame can generate ghost detections or fragment a single cheese slice into multiple clusters. PCL's `StatisticalOutlierRemoval` filter removes points whose distance to their neighbours deviates beyond a statistical threshold — a standard and lightweight operation that makes the downstream segmentation reliable under real factory conditions.

__Technologies:__ OpenCV, PCL (Point Cloud Library), ROS2 `sensor_msgs/PointCloud2`

**Why not AI detection:** Cheese slices are geometrically simple and consistent within a batch. RANSAC + clustering reliably separates them from the conveyor surface. Adding a neural network would increase latency, require training data, and add infrastructure complexity without benefit for this use case.

---

### 5.3 TF2 Transform Manager

Converts all coordinates from camera frame to robot base frame.

```sh
P_robot = T_robot←camera × P_camera
```

Calibration is performed once at startup using a calibration target. The transform is published as a static TF. All downstream nodes operate exclusively in the robot base frame.

---

### 5.4 Conveyor State Estimator

**The most time-critical module.**

The conveyor runs at a constant known speed. After initial detection by the camera, the object position is propagated using time elapsed and the known belt speed:

```
t=0:  detect object at x₀ (camera)
t=T:  x(T) = x₀ + v_belt × (T - t₀)
```

Or equivalently:

```
x(t) = x_detected + conveyor_speed × (t - t_detection)
```

This eliminates the need to re-detect the object in every camera frame. One detection is sufficient — the position is tracked deterministically until the robot picks it.

If a conveyor encoder is available, the Conveyor State Estimator can additionally use encoder feedback to improve long-term tracking accuracy and compensate for mechanical drift. The architecture supports both modes — constant speed assumption or encoder-assisted — without changes to any other component.

---

### 5.5 Object Queue

The camera detects cheese at 30 frames per second. The robot picks cheese once every 4–6 seconds. These two rates are completely different. The queue sits between them as a buffer so neither side has to wait for the other — the camera keeps detecting regardless of what the robot is doing, and the robot takes the next item when it is ready regardless of what the camera is doing.

```
Camera (30 Hz)                     Robot (1 pick per 4-6 sec)
      │                                      │
      ▼                                      │
detects C1 → [C1]                            │
detects C2 → [C1, C2]                        │
detects C3 → [C1, C2, C3]                    │
                           ←──────── robot ready → picks C1
             [C2, C3]                         │
                           ←──────── robot ready → picks C2
             [C3]
```

**Maximum queue size**

The queue size is not an arbitrary number — it is bounded by physics. The maximum number of cheese pieces that can simultaneously be inside the robot's reachable workspace is:

```
max_queue_size = workspace_length / min_cheese_spacing

workspace_length   = length of conveyor segment the robot can reach over
min_cheese_spacing = conveyor_speed × min_arrival_interval
```

Example with conveyor speed 100mm/s and minimum arrival interval 4 seconds:

```
min_cheese_spacing = 100 × 4 = 400mm
workspace_length   = 400mm  (robot reach over conveyor)
max_queue_size     = 400 / 400 = 1–2 pieces at a time
```

In practice with 4–6 second arrival intervals the queue will rarely hold more than 2–3 pieces. The max queue size is measured from the physical setup, not guessed.

**Deadline policy — when to discard a queued item**

Each queued item receives a deadline at detection time — the last moment the robot can still intercept it before it exits the workspace:

```
deadline = t_detection + (workspace_exit_x - x_detected) / conveyor_speed
```

Every item is discarded automatically when its deadline passes. This is the expiry mechanism — no manual cleanup needed.

**Priority policy**

The queue is FIFO — First In, First Out. The piece that arrived first has the earliest deadline and is always picked first. No reordering is needed.

```
Queue: [C1 deadline=t+1.2s,  C2 deadline=t+5.8s,  C3 deadline=t+9.1s]
         ↑ always pick this first
```

| Policy | Rule |
|---|---|
| Max size | workspace_length ÷ min_cheese_spacing |
| Discard when | `now ≥ deadline` OR robot cannot finish pick before deadline |
| Priority | FIFO — earliest deadline first |
| On discard | Log missed pick, cheese falls to reject bin, continue with next |

---

### 5.6 Task Scheduler

The central coordinator. Combines object queue state, container state, and robot availability to decide the next action.

**Feasibility check before every pick:**

The scheduler evaluates two conditions before committing to a pick:

```
1. Is the object still alive?
   now < deadline           →  YES, still pickable
   now ≥ deadline           →  NO,  discard immediately

2. Can the robot finish the pick before the deadline?
   now + t_plan + t_execute < deadline  →  YES, schedule it
   now + t_plan + t_execute ≥ deadline  →  NO,  skip it
```

If either condition fails the item is popped from the queue, logged as a missed pick, and the next queued item is evaluated immediately.

**Missed pick policy:** The reject bin beneath the conveyor catches any piece the robot could not reach. The belt does not stop. The scheduler moves to the next item without delay.

---

### 5.7 Container Tracker + Index Controller

Tracks the packaging state of the container grid.

```
Container (2×4 grid):
┌────┬────┬────┬────┐
│ ✓  │ ✓  │    │    │  ← row 0
├────┼────┼────┼────┤
│    │    │    │    │  ← row 1
└────┴────┴────┴────┘
```

**Is the container advancement automatic?** Yes — fully automated, no operator involvement. When the Container Tracker detects that all slots in a row are filled, it fires a row-complete event. The Container Index Controller receives that event and immediately sends a command to advance the container conveyor to the next row. The operator never needs to press a button.

```
Place completed → Container Tracker updates slot (0,3) → row 0 full
    → fires ROW_COMPLETE event
    → Container Index Controller sends advance command
    → container moves forward
    → row 1 is now under the robot
    → system continues picking
```

Tracking and control are kept as separate components — the tracker is pure state (which slots are filled), the controller handles the physical actuation (moving the container). This separation makes it easier to test and maintain each part independently.

---

### 5.8 Pick Planner

Computes exactly where the robot gripper must be positioned to pick the cheese and where it must place it in the container.

**Pick pose — where to grab:**

The Pick Planner answers: "where exactly should the robot position its gripper to grab this cheese slice?"

**Antipodal grasp** means the two gripper fingers press against opposite faces of the cheese slice simultaneously — one finger on the left face, one on the right. This is the most stable grasp for a flat rectangular object.

**Yaw-aligned** means the gripper rotates to match the orientation of the cheese slice on the belt. If the slice is lying at an angle, the gripper rotates to the same angle before descending — otherwise the fingers would hit the edge instead of the flat face.

```
Wrong — gripper at 0°, cheese at 45°:    Correct — gripper matches cheese:

    ║   ║                                     ╱   ╱
    ║   ║                                    ╱   ╱
  ┌──────┐  ← hits corner                  ╱─────╱  ← clean grip
  │CHEESE│                                ╱CHEESE╱
  └──────┘                               ╱─────╱
```

**Place pose — where to release:**

The Container Tracker says "next slot is row 0, column 2". The Pick Planner converts that into a physical (x, y, z) position by computing:

```
place_x = container_origin_x + column × slot_spacing_x + container_current_offset
place_y = container_origin_y + row    × slot_spacing_y
place_z = container_height + descent_clearance
```

The **container offset** accounts for the container's current position since the container itself moves rhythmically. Without this the cheese would land next to the slot instead of inside it.

```
Container grid:
┌────┬────┬────┬────┐
│ ✓  │ ✓  │ ←  │    │  slot (0,2) = container_origin + 2 × slot_spacing
└────┴────┴────┴────┘
```

**Outputs:**

- Pick pose: `{x, y, z, yaw}` above object + descent offset
- Place pose: `{x, y, z}` computed from container slot index + container offset

---

### 5.9 MoveIt2 Planner

Uses the provided xArm6 URDF directly. Responsibilities:

- Inverse kinematics solving
- Collision-aware trajectory generation (avoids conveyor, container, robot self-collision)
- Cartesian path planning for approach/descent/retract phases

**Pick trajectory phases:**

```
Home → Pre-pick (above object) → Pick (descend) → Grasp → 
Retract → Pre-place (above slot) → Place (descend) → Release → Home
```

**Dynamic 3D workspace modelling — OctoMap:**

The Orbbec camera feed is used not only for cheese detection but also to continuously build a live 3D occupancy map of the workspace using MoveIt2's OctoMap integration. Every depth frame updates the map, representing occupied vs. free space in a voxel grid.

This serves two purposes:

1. **Static collision objects** — the conveyor belt, container edges, and mounting structures are represented as permanent collision bodies in the planning scene
2. **Dynamic obstacle avoidance** — if an unexpected object enters the workspace (a hand, a fallen piece, a tool), MoveIt2 detects the new occupied voxels and automatically replans a safe trajectory around the obstacle in real time rather than halting the system

```
Orbbec depth stream
      │
      ▼
OctoMap (live 3D occupancy grid)
      │
      ▼
MoveIt2 Planning Scene
      │
      ▼
Collision-aware trajectory replanning
```

This makes the system robust to real factory conditions where unexpected objects occasionally enter the robot's workspace.

---

### 5.10 Robot Controller

Executes MoveIt2 trajectories on the xArm6 hardware. Controls the mechanical gripper. Returns execution feedback to the Task Scheduler.

Feedback events:

```
PICK_STARTED
GRIPPER_CLOSED
PICK_COMPLETED
PLACE_COMPLETED
GRIPPER_OPENED
MOTION_FAILED
```

**Safety stop and automated homing:**

MoveIt2 handles path planning but does not own the recovery logic. The Robot Controller implements a supervisor that responds to fault conditions:

```
Force/torque spike detected  OR  collision flag raised
              │
              ▼
      IMMEDIATE SAFETY STOP
      (all joints halted)
              │
              ▼
      CONTROLLED HOMING SEQUENCE
      (MoveIt2 plans path to known safe home position)
              │
              ▼
      READY state published to Task Scheduler
      (system can resume or await operator confirmation)
```

This ensures the robot never remains frozen mid-motion after a fault. The homing sequence returns the arm to a known state from which normal operation can safely resume or an operator can inspect the situation.

---

## 6. System State Machine

```ini
              ┌───────────────────┐
              │   UNCONFIGURED    │
              └────────┬──────────┘
                       │  operator loads batch recipe
              ┌────────▼──────────┐
              │ BATCH_CONFIGURATION│
              └────────┬──────────┘
                       │  configuration complete
              ┌───────▼───────┐
         ┌───▶│    ACTIVE     │◀────────────────┐
         │    └───────┬───────┘                 │
         │            │  object detected         │
         │    ┌───────▼───────┐                 │
         │    │  SCHEDULING   │                 │
         │    └───────┬───────┘                 │
         │            │                          │
         │     feasible?                         │
         │    ┌────────┴────────┐                │
         │    ▼                 ▼                │
         │  YES                NO                │
         │    │                 │                │
         │    ▼                 ▼                │
         │  PLANNING        SKIP_OBJECT──────────┘
         │    │
         │    ▼
         │  EXECUTING
         │    │
         │  ┌─┴──────────┐
         │  ▼             ▼
         │ SUCCESS    MISSED_PICK
         │  │             │
         │  └──────┬──────┘
         │         ▼
         └─────── ACTIVE

   ANY STATE ──→ CRITICAL_FAULT ──→ SAFE_STOP
```

---

## 7. Fault Handling

### Fault Classification

| Type | Examples | Action |
|---|---|---|
| Non-critical | Missed pick, object exits workspace, gripper slip | Log, increment counter, continue |
| Critical | Camera offline, robot offline, encoder failure, E-stop | Safe stop, alert |

### Health Monitor

Every node publishes a heartbeat on `/diagnostics`. Silence for > 3 seconds triggers a fault.

```ini
Vision Node     ──/diagnostics──▶
Scheduler       ──/diagnostics──▶  Fault Manager
Robot Controller──/diagnostics──▶
```

### Fault Manager Flow

```ini
System Event
     │
     ▼
Fault Manager
     │
 ┌───┴───┐
 ▼       ▼
Warn   Critical
 ▼       ▼
Log   Safe Stop
 ▼
Continue
```

---

## 8. Observability

Foxglove Studio is the primary operational interface, accessed remotely over VPN via WebSocket without heavy desktop software on the server.

| Panel | Content |
|---|---|
| 3D view | Robot model (URDF), conveyor TF frame, detected objects as markers, planned trajectories, OctoMap occupancy |
| Image panel | Live Orbbec RGB feed with detection overlay |
| Plot panel | Cycle time, queue depth, missed pick rate |
| Log panel | Scheduler decisions, fault events categorized by severity |
| Diagnostics | Node heartbeat status |

**Telemetry categorization:**

All events published to Foxglove are classified into three tiers:

| Tier | Examples | Action |
|---|---|---|
| **Info** | Successful pick, slot filled, container advanced, cycle time within nominal range | Logged, shown in green, counted in metrics |
| **Warning** | Latency spike, near-miss (object barely reached), queue growing beyond expected depth, missed pick | Logged, shown in yellow, operator notified if repeated |
| **Critical** | Collision detected, camera offline, robot unresponsive, E-stop triggered, homing failed | Logged, shown in red, system halted, operator intervention required |

This three-tier structure makes the Foxglove dashboard actionable — an operator can immediately distinguish between normal operation noise, conditions that need monitoring, and conditions that require immediate action.

---

## 9. Development and Deployment

### Target Environment

- **Hardware server:** Ubuntu Server, mini server at Estica facility
- **Remote access:** VPN + SSH, IDE via remote tunnel (VS Code Remote SSH)
- **Runtime:** ROS2 Humble, no Docker on server (native install)
- **Visualization:** Foxglove Studio on local Mac via WebSocket bridge

### Development Workflow

```ini
Local Mac (dev)
    → SSH tunnel → Ubuntu Server
    → run ROS2 nodes
    → Foxglove Studio (localhost:8765)
    → observe live
```

### Testing Strategy

1. **Unit testing per node** — vision, tracking, scheduler independently
2. **Full simulation in Foxglove** — complete end-to-end pipeline validated before touching real hardware (see below)
3. **Bag file replay** — record one real session, replay for regression testing
4. **Hardware-in-loop** — full pipeline on real robot before production

---

### Foxglove Simulation

Before real hardware is available the entire system is validated in simulation. Three lightweight mock publisher nodes replace the physical hardware interfaces. The higher-level planning and scheduling logic runs unchanged — only the hardware interfaces are replaced by simulation publishers.

| Mock Node | Replaces | What it publishes |
|---|---|---|
| `conveyor_sim` | Real conveyor belt | Belt position, cheese detections at random 4–6 sec intervals |
| `container_sim` | Real container conveyor | Container position, current grid state |
| `camera_sim` | Orbbec Astra camera | Fake cheese poses at realistic positions with batch dimensions |

**What is visible in Foxglove during simulation:**

```
3D panel:
  - Conveyor belt moving forward (TF frame)
  - Cheese slices appearing at random intervals as 3D box markers
  - Each marker moving forward with the belt at constant speed
  - Robot arm planning and moving to intercept each slice
  - Gripper closing, picking, placing into the correct grid slot
  - Container grid filling slot by slot
  - Row completes → container advances → new empty row appears
  - Process repeats continuously

Plot panel:
  - Queue depth over time
  - Cycle time per pick
  - Missed pick count

Log panel:
  - Scheduler decisions (pick / skip / missed)
  - Container slot assignments
```

This is the approach Guillaume recommended when he said *"use Foxglove for visualization and end-to-end testing."* The simulation proves the full system logic works correctly before a single cable is connected to the real robot.

---

## 10. Time Estimate

Two milestones with distinct scopes:

| Milestone | Duration | Description |
|---|---|---|
| **MVP — Simulation** | 4 weeks | Full pipeline working in Foxglove simulation. All logic validated. No real hardware required. |
| **Production-ready** | 8–10 weeks | Deployed on real robot, real conveyor, real camera. Validated under continuous operation. |

This distinction is important. Software working in simulation and software validated on industrial hardware are two different deliverables with different risk profiles.

**Detailed phase breakdown:**

| Phase | Duration | Deliverables |
|---|---|---|
| **1. Infrastructure** | Week 1–2 | Ubuntu server setup, ROS2 install, Orbbec SDK integration, Foxglove pipeline, xArm6 URDF loaded and visualized |
| **2. Perception** | Week 3–4 | Point cloud processing, object segmentation, 3D bounding box, TF2 camera calibration, conveyor tracking |
| **3. Motion Planning** | Week 5–6 | MoveIt2 config for xArm6, IK validation, trajectory planning, gripper control, pick/place trajectory phases |
| **4. System Integration** | Week 7–8 | Task scheduler, container tracker, state machine, batch configuration, end-to-end single pick working |
| **5. Testing and Hardening** | Week 9–10 | Fault handling, missed pick recovery, cycle time optimization, continuous operation testing, Foxglove dashboard |

> **MVP is achievable in 4 weeks** (phases 1–2 + simulation). Full production deployment requires 8–10 weeks including hardware integration and industrial validation.

### Assumptions

- xArm6 hardware available for testing from week 5 onwards
- Conveyor belt and container system available from week 7 onwards
- Orbbec camera mounting position fixed and stable

### Risk Factors

- **Camera calibration accuracy** — directly affects pick precision; may require iteration
- **Conveyor speed calibration** — the belt speed must be accurately measured once at setup; any drift over time will affect pick accuracy
- **Gripper fingertip design** — mechanical fit for different cheese sizes may require physical iteration

---

### Throughput and Scalability

The system's throughput ceiling is determined by hardware physics, not software. Once software latency is minimized, the bottleneck becomes the robot arm's cycle time — how fast it can complete one full pick-and-place motion.

```
Throughput ceiling = 1 / (pick_time + place_time + return_time)

Example: 1.5s pick + 1.5s place + 1.0s return = 4s cycle
         → max 15 picks/min with one arm
```

If the required throughput exceeds what a single arm can deliver, scaling requires structural changes — not software tuning:

| Scaling option | Description |
|---|---|
| **Upstream buffer zone** | Add a longer camera detection zone before the robot workspace, giving the scheduler more time to plan and reducing missed picks |
| **Second robot arm** | Add a parallel arm working on the same conveyor, doubling throughput without changing any software logic — each arm gets its own queue and scheduler instance |

Pushing a single arm faster than its physical limit causes missed picks, not higher throughput. Scaling is a hardware decision.

---

---

## 11. ROS2 Communication Map

All components communicate exclusively through ROS2 interfaces. No direct function calls between nodes.

| Node | Publishes | Subscribes to | Interface |
|---|---|---|---|
| Vision Processing | `/detected_objects` | `/camera/depth/points`, `/camera/color/image_raw` | Topic |
| Conveyor State Estimator | `/tracked_objects` | `/detected_objects` | Topic |
| Object Queue | `/queue_state` | `/tracked_objects` | Topic |
| Task Scheduler | `/pick_task` | `/queue_state`, `/container_state`, `/execution_feedback` | Topic |
| Pick Planner | `/pick_plan` | `/pick_task` | Action |
| MoveIt2 Planner | `/joint_trajectory` | `/pick_plan` | Action |
| Robot Controller | `/execution_feedback`, `/joint_states` | `/joint_trajectory` | Action + Topic |
| Container Tracker | `/container_state` | `/execution_feedback` | Topic |
| Container Index Controller | `/container_cmd` | `/container_state` | Topic |
| Calibration Manager | `/batch_config` | operator service call | Service |
| Health Monitor | `/diagnostics` | all node heartbeats | Topic |

**Interface types:**

- **Topic** — fire and forget, continuous data stream (detections, joint states, diagnostics)
- **Service** — request/response, short operations (batch configuration, parameter queries)
- **Action** — long-running operations with feedback and cancellation (pick planning, trajectory execution)

---

*Architecture designed for ROS2 Humble. All components are ROS2 nodes communicating via topics, services, and actions.*
