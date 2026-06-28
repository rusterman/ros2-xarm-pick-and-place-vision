# Package Structure — Why the Repository Is Organized This Way

**Author:** Rustam
**Date:** June 2026
**Version:** 1.0

---

## Table of Contents

1. [Purpose and Scope](#1-purpose-and-scope)
2. [Core Principle: Separation by Responsibility](#2-core-principle-separation-by-responsibility)
3. [Repository Layout](#3-repository-layout)
4. [Layer-by-Layer Breakdown](#4-layer-by-layer-breakdown)
5. [The Clean Architecture Mapping](#5-the-clean-architecture-mapping)
6. [Worked Example: Container Indexer Sim/Real Swap](#6-worked-example-container-indexer-simreal-swap)
7. [Why the Conveyor Is Treated Differently](#7-why-the-conveyor-is-treated-differently)
8. [Why Simulation Is Not Mocked](#8-why-simulation-is-not-mocked)
9. [Physics Fidelity — Weight, Gravity, and Force Feedback](#9-physics-fidelity--weight-gravity-and-force-feedback)
10. [Naming Convention](#10-naming-convention)
11. [Enforcing the Boundary](#11-enforcing-the-boundary)
12. [Open Decisions](#12-open-decisions)

---

## 1. Purpose and Scope

[ARCHITECTURE.md](ARCHITECTURE.md) describes **what the system does** — the processing pipeline, the state machine, the node communication map. This document describes **how the code that implements that system is split into ROS2 packages**, and why. It assumes the node names and responsibilities defined in ARCHITECTURE.md §5 and §11.

Nothing in this document is built yet — it is the agreed package layout to scaffold against, not a description of existing code.

---

## 2. Core Principle: Separation by Responsibility

Every package falls into exactly one of five responsibilities. A package must never mix them:

| Responsibility | Question it answers | May depend on | Must never depend on |
|---|---|---|---|
| **Description** | "What does this object look like, and what are its joints?" | `xacro`, `robot_state_publisher` | any logic, any node code |
| **Interfaces** | "What does a message/service/action look like?" | nothing (pure `.msg`/`.srv`/`.action`) | anything else in the repo |
| **Functional logic** | "What decision should the system make?" | `estica_interfaces`, standard ROS2 messages | `gazebo_*`, `ros_gz_*`, vendor SDKs |
| **Hardware** | "How is a decision turned into motion — sim or real?" | `estica_interfaces`, vendor SDK, or `gazebo_ros2_control` | functional-layer packages |
| **Bringup / Simulation** | "What gets started, and what does the simulated world contain?" | everything — it's the integration point | — |

The rule that makes the whole structure work: **dependencies only point toward Functional logic, never away from it.** Functional packages are the stable center; everything else — geometry, message wire format, simulated or real actuation — is a replaceable detail wrapped around that center. This is the same rule Clean Architecture calls the **Dependency Rule** (§5).

---

## 3. Repository Layout

```
src/
├── description/                       DATA ONLY — zero nodes, zero logic
│   ├── xarm_description/
│   ├── xarm_gripper/
│   ├── orbbec_description/
│   ├── conveyor_description/
│   ├── container_description/
│   ├── cheese_description/
│   └── cell_description/              composes all of the above into one scene
│
├── interfaces/
│   └── estica_interfaces/             /pick_task, /container_cmd, /batch_config, …
│
├── bringup/
│   └── estica_bringup/                launch files only
│
├── simulation/
│   └── estica_gazebo/                 world file + cheese spawner node only
│
├── hardware/
│   └── container_hardware/            ros2_control hardware interface
│
├── functional/                        CORE LOGIC — never imports gazebo_*
│   ├── vision_processing/
│   ├── conveyor_tracking/             (state estimator + object queue)
│   ├── task_scheduler/
│   ├── pick_planner/
│   ├── estica_moveit_config/
│   ├── robot_controller/
│   ├── container_control/             (tracker + index controller)
│   └── calibration_manager/
│
└── operational/
    └── estica_diagnostics/            (health monitor + fault manager)
```

---

## 4. Layer-by-Layer Breakdown

### `description/` — geometry, nothing else

Each package is pure URDF/xacro + meshes. No package in this folder may contain a node, a launch file with logic, or a runtime dependency beyond `xacro`/`robot_state_publisher`. `cell_description` is the one exception that *composes*: it includes every other description and defines the fixed joints placing them relative to each other (camera above the conveyor, container past the conveyor's far end, etc.) — it still contains zero behavior, only static placement.

### `interfaces/` — the contract

`estica_interfaces` holds every custom message, service, and action type referenced in ARCHITECTURE.md §11 (`/pick_task`, `/pick_plan`, `/container_cmd`, `/batch_config`, …). Every other package that needs to talk to another package depends on this one — it is the only thing allowed to sit between Functional and everything else.

### `functional/` — the actual business rules

One package per independently-testable node, matching ARCHITECTURE.md §9's own testing strategy ("unit testing per node — vision, tracking, scheduler independently"). Two pairs are merged because they are always deployed together and described as a unit in ARCHITECTURE.md: `conveyor_tracking` (state estimator + object queue, §5.4–5.5) and `container_control` (tracker + index controller, §5.7). These packages may depend on `estica_interfaces` and standard ROS2 message types. They must never depend on `gazebo_ros`, `ros_gz`, or any vendor SDK — if a functional package needs to know whether it's running in simulation, that is itself a design smell.

### `hardware/` — the actuation boundary

Where Functional logic ends and physical motion begins. A hardware package implements a `ros2_control` hardware interface with two interchangeable backends: `gazebo_ros2_control` for simulation, a vendor driver for the real motor/PLC. Functional packages talk to the generic `ros2_control` controller interface and never know which backend is loaded.

### `simulation/` + `bringup/` — the integration layer

`estica_gazebo` is not a mock-data package — see §8. It owns the Gazebo world file and the one genuinely simulation-only component: the cheese spawner (random size + pose at the belt's start, with no real-world software equivalent to swap to). `estica_bringup` contains only launch files: it decides, per launch argument, whether `estica_gazebo` + `gazebo_ros2_control` or the real camera driver + real motor driver get started underneath the same Functional layer.

### `operational/` — cross-cutting observability

`estica_diagnostics` implements the Health Monitor and Fault Manager from ARCHITECTURE.md §7–8. Every other node publishes a heartbeat to it; it does not sit in the main pipeline and nothing depends on it.

---

## 5. The Clean Architecture Mapping

Every label below is either a package name already defined in §3, or a section of `ARCHITECTURE.md` cited directly — nothing here is invented. Read top to bottom = outermost to innermost. The Dependency Rule: every arrow points *down* — nothing inside a box may import or know about anything in a box above it.

```
┌───────────────────────────────────────────────────────────────────┐
│ FRAMEWORKS & DRIVERS   (outermost - most volatile)                │
│                                                                   │
│ estica_gazebo, description/ packages, hardware/container_hardware │
│ MoveIt2 solver, real camera/motor/PLC drivers                     │
└───────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼  depends on
┌───────────────────────────────────────────────────────────────────┐
│ INTERFACE ADAPTERS                                                │
│                                                                   │
│ estica_interfaces (the message/service/action contracts)          │
│ ROS2 node-wrapper code inside each functional/ package            │
└───────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼  depends on
┌───────────────────────────────────────────────────────────────────┐
│ USE CASES   (the decision logic in each functional/ package)      │
│                                                                   │
│ task_scheduler, pick_planner, container_control,                  │
│ calibration_manager, conveyor_tracking, vision_processing         │
└───────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼  depends on
┌───────────────────────────────────────────────────────────────────┐
│ ENTITIES   (innermost - the data ARCHITECTURE.md already defines) │
│                                                                   │
│ detected-object data (ARCHITECTURE.md section 5.2),               │
│ container grid state (5.7), batch recipe (5.1), deadline (5.5)    │
└───────────────────────────────────────────────────────────────────┘
```

| Ring | The real thing it maps to | Where |
|---|---|---|
| **Frameworks & Drivers** | `estica_gazebo`, every `description/` package, `hardware/container_hardware`, MoveIt2's IK solver, real camera/motor/PLC drivers | `simulation/`, `description/`, `hardware/` |
| **Interface Adapters** | `estica_interfaces` (the actual message/service/action definitions — `/pick_task`, `/container_cmd`, `/batch_config` from [ARCHITECTURE.md §11](ARCHITECTURE.md#L765)), plus the ROS2 node-wrapper code (topic/service/action callbacks) inside each package below | `interfaces/estica_interfaces`, the node file inside each `functional/*` package |
| **Use Cases** | The decision logic each functional package already owns: `task_scheduler`'s feasibility check ([§5.6](ARCHITECTURE.md#L349)), `pick_planner`'s pick/place pose computation ([§5.8](ARCHITECTURE.md#L401)), `container_control`'s row-complete detection + advance command ([§5.7](ARCHITECTURE.md#L373)), `calibration_manager`'s batch reconfiguration ([§5.1](ARCHITECTURE.md#L150)), `conveyor_tracking`'s position prediction + deadline policy ([§5.4–§5.5](ARCHITECTURE.md#L260)), `vision_processing`'s segmentation pipeline ([§5.2](ARCHITECTURE.md#L212)) | each `functional/*` package |
| **Entities** | The plain data ARCHITECTURE.md already names: the detected-object schema `{x, y, z, yaw, length, width, height}` ([§5.2](ARCHITECTURE.md#L237)), the container grid's filled/empty slot state ([§5.7](ARCHITECTURE.md#L373)), the batch recipe ([§5.1](ARCHITECTURE.md#L192)), and the per-item deadline ([§5.5](ARCHITECTURE.md#L321)) | carried as fields inside `estica_interfaces` messages — there is no separate "entities package" |

**Why no separate Entities package:** in this system the Entities ring isn't a new package — it's just the *data fields* already carried inside `estica_interfaces` messages. There's no behavior to separate out beyond what the Use Case packages above already do with that data.

**Where Controllers, Presenters, and Gateways fit, specifically:**

- **Controller** = the part of a `functional/*` package's node that receives a topic/service/action call and invokes that package's own decision logic — e.g. `task_scheduler`'s node receiving `/queue_state` and `/container_state` and calling its feasibility check.
- **Presenter** = the part that takes that decision and formats it outward — e.g. publishing `/pick_task`, or writing a line to the Foxglove log panel.
- **Gateway** = an interface a Use Case calls for something it needs but doesn't own — e.g. `pick_planner` asking for the current TF lookup. Most of this is already free in ROS2: a topic name + message type *is* the abstract interface, so swapping `estica_gazebo` for real hardware doesn't require any Use Case package to change. A Gateway is only worth writing explicitly where a Use Case needs something synchronous that isn't naturally pub/sub — e.g. a direct TF2 buffer call — so it can be swapped for a fake in a unit test.

Dependency direction: outer rings depend on inner rings, never the reverse. Swapping `estica_gazebo` for real hardware in `estica_bringup` should not require changing a single line inside `task_scheduler`, `pick_planner`, `container_control`, `calibration_manager`, `conveyor_tracking`, or `vision_processing`. That property — not the folder names — is what makes this "clean architecture" rather than just a tidy directory listing.

**Honest caveat:** ROS2's "ports" are topics/services/actions, not compiler-checked interfaces like in textbook Clean Architecture. The boundary between rings is enforced by what each file imports and what each `package.xml` declares, not by the type system. It works, but it relies on discipline — see §11.

---

## 6. Worked Example: Container Indexer Sim/Real Swap

```
container_control          ← decision logic ("row is full, advance now")
        │
        │  sends a command through ros2_control's
        │  controller_manager — a generic interface
        ▼
container_hardware          ← the swappable plugin
        │
   ┌────┴────┐
   ▼         ▼
Gazebo    real motor
joint     driver / PLC
(sim)     (production)
```

`container_hardware` implements `read()`/`write()` once. The sim backend (`gazebo_ros2_control`) moves a Gazebo joint; the real backend talks to the actual indexer motor. `container_control` never imports either — it only speaks the generic `ros2_control` interface, so the exact same compiled node runs unmodified in simulation and in production.

---

## 7. Why the Conveyor Is Treated Differently

The conveyor does **not** get a `ros2_control` hardware interface. ARCHITECTURE.md states the belt runs at "constant speed, no encoder required" ([line 77](ARCHITECTURE.md#L77)), and the Communication Map (§11) has no topic that commands it — it is a fixed-speed motor entirely outside the ROS2 loop, only ever read as a known constant by the Conveyor State Estimator.

Building a hardware-interface abstraction for an actuator the software never actually commands buys nothing: there is no real backend to validate parity against. The conveyor's belt motion and the cheese moving along it are therefore plain scripted entity updates inside `estica_gazebo`, not a `ros2_control` package. This is a deliberate asymmetry, not an oversight — `ros2_control` is reserved for actuators with a genuine software-issued command path (the container indexer's `/container_cmd`), where sim/real code parity actually means something.

---

## 8. Why Simulation Is Not Mocked

The original plan (now corrected — see ARCHITECTURE.md §9.3) had a `camera_sim` node publish fabricated cheese poses directly, skipping `vision_processing` entirely. That validates the scheduler and planner, but never tests the part of the system most likely to fail in production: vision.

The corrected design has no `camera_sim` node at all. The Orbbec's existing Gazebo sensor plugin renders real RGB-D frames of the actual simulated scene — real spawned cheese geometry, real conveyor and container models — and publishes them as standard `sensor_msgs/Image` + `PointCloud2`, identical to what the real camera driver produces. `vision_processing` runs against this completely unmodified. The same logic applies to conveyor and container state: both are real spawned/moved Gazebo entities, tracked the same way real footage would be, never injected as fake topic data.

---

## 9. Physics Fidelity — Weight, Gravity, and Force Feedback

Sensor realism (§8) is only half of "realistic Gazebo simulation." The other half is physical realism: does the simulated cheese behave like a real object with real mass under gravity, and can the system observe the consequences of that.

### Cheese mass must be derived, not hardcoded

`cheese_description`'s size is parametrized via xacro args (radius/height) tied to the same batch dimensions used in Batch Calibration ([ARCHITECTURE.md §5.1](ARCHITECTURE.md#L192)). Its `<inertial>` block must scale from those same args through a density constant (`mass = density × volume`), not a fixed number — otherwise resizing a batch silently leaves the simulated weight wrong. A link with zero or placeholder mass in Gazebo either behaves as effectively static or produces unrealistic contact dynamics (jitter, ignoring gravity, flying off on contact), so this is required for the physics to mean anything, not a cosmetic detail.

### Gravity and collision are implicit — but must be a stated requirement

Gazebo applies gravity and collision dynamics to any link with proper `<inertial>` + `<collision>` geometry by default; no extra package or plugin makes this exist. Because it's implicit, it's easy to end up with a `cheese_description` that has visual geometry but no working physics (missing collision geometry, placeholder inertia). Stating it here means it can't be silently skipped: every spawned cheese needs real collision geometry and mass-derived inertia, not just a mesh.

### What is, and isn't, observable in Foxglove

Foxglove does not render Gazebo physics directly — it only shows what's published to a ROS2 topic:

| Physical effect | Observable in Foxglove? | How |
|---|---|---|
| Cheese position/orientation under gravity, sliding, tipping | Yes, automatically | The camera renders wherever physics actually placed the cheese; this flows through the existing point cloud/image pipeline (§8) with no extra plugin |
| Gripper force vs. cheese weight (did grip strength match the simulated mass) | Not yet | Requires a force/torque sensor plugin on the gripper in Gazebo, publishing to a topic the Robot Controller's fault path ([ARCHITECTURE.md §5.10](ARCHITECTURE.md#L510), "Force/torque spike detected") can react to and Foxglove's plot panel can chart |

The first case needs nothing further — it's a free consequence of routing simulation through a real sensor plugin (§8). The second is a real gap: without a simulated force/torque sensor, the safety-stop fault path in ARCHITECTURE.md §5.10 has nothing realistic to trigger on in simulation, so that part of the system would ship untested until real hardware arrives. This belongs as a `<gazebo>` sensor plugin on the gripper link inside `xarm_gripper` — it's sensing, not actuation, so it stays in `description/` rather than `hardware/`.

---

## 10. Naming Convention

- **`estica_` prefix** — used for cross-cutting, integration-level packages whose names would otherwise be dangerously generic (`interfaces`, `bringup`, `diagnostics`, `gazebo`, `moveit_config`). Without a prefix these would risk colliding with identically-named packages from other ROS2 workspaces.
- **No prefix** — used for packages that describe one unambiguous physical object (`xarm_description`, `conveyor_description`, `container_hardware`, …). The object name alone is already specific enough; a prefix would only add noise.

---

## 11. Enforcing the Boundary

The dependency rule in §2 is currently a convention, not a mechanical check. Once CI exists, the cheapest possible guardrail is a one-line lint: grep every `functional/*/package.xml` for `gazebo`, `ros_gz`, or `ign` — a match means the boundary has been violated. This is the kind of rule that erodes silently over months (a quick debug import that's never removed), so it is worth automating as soon as there is a CI pipeline to run it in.

---

## 12. Open Decisions

- This layout is agreed but **not yet scaffolded** — no package in `description/`, `hardware/`, `functional/`, `simulation/`, or `operational/` exists yet except the pre-existing `xarm_description`, `xarm_gripper`, and `orbbec_description`.
- Whether any `functional/` packages should be split further once implementation reveals tighter or looser coupling than expected (e.g. `conveyor_tracking` may turn out to need separating).
- Whether `estica_moveit_config` is generated via MoveIt Setup Assistant before or after `cell_description` stabilizes, since the generated config depends on final link/joint names.
