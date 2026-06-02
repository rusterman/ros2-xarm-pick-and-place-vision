# ROS2 Pub/Sub Architecture & RViz2 — Big Picture

## Table of Contents
1. [The Big Picture](#the-big-picture)
2. [Core Concepts](#core-concepts)
3. [How Nodes Communicate (DDS)](#how-nodes-communicate-dds)
4. [Node-by-Node Breakdown](#node-by-node-breakdown)
5. [How RViz2 Fits In](#how-rviz2-fits-in)
6. [Event Loop](#event-loop)
7. [Full System Flow](#full-system-flow)

---

## The Big Picture

In ROS2 every program is a **node** — an independent process with a name. Nodes don't call each other directly. Instead they communicate through **topics**: named message channels that anyone can publish to or subscribe from.

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2 Network (DDS)                       │
│                                                                 │
│  ┌──────────────────┐   /hello_topic    ┌───────────────────┐  │
│  │  HelloPublisher  │ ────────────────► │  HelloSubscriber  │  │
│  └──────────────────┘                   └───────────────────┘  │
│                                                                 │
│  ┌──────────────────┐  /visualization_  ┌───────────────────┐  │
│  │ MarkerPublisher  │     marker        │      RViz2        │  │
│  └──────────────────┘ ────────────────► │   (also a node)   │  │
│                                         └───────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

Key principle: **nodes don't know who is listening**. A publisher just sends data to a topic. Zero, one, or many subscribers can receive it — the publisher doesn't care.

---

## Core Concepts

| Concept | What it is |
|---|---|
| **Node** | A single running program in the ROS2 graph. Has a name (e.g. `hello_publisher`). |
| **Topic** | A named channel for messages (e.g. `/hello_topic`). Has a fixed message type. |
| **Publisher** | A node endpoint that sends messages onto a topic. |
| **Subscription** | A node endpoint that receives messages from a topic. |
| **Message type** | The data schema. `String` is plain text. `Marker` is a 3D visual object. |
| **Queue depth** | How many messages to buffer if the subscriber is slow (e.g. `10`). |
| **DDS** | The underlying transport layer that matches publishers to subscribers automatically. |
| **Event loop** | `rclpy.spin(node)` — keeps a node alive and fires callbacks when events arrive. |

---

## How Nodes Communicate (DDS)

ROS2 uses **DDS (Data Distribution Service)** as its transport. There is no central broker or server.

**Auto-discovery works like this:**

```
1. HelloPublisher starts
   → announces: "I publish std_msgs/String on /hello_topic"

2. HelloSubscriber starts
   → announces: "I want std_msgs/String from /hello_topic"

3. DDS sees the match
   → opens a direct peer-to-peer connection between them

4. Messages flow directly: publisher → subscriber
   (no middleman)
```

This means:
- Nodes can start and stop in any order
- Multiple subscribers can receive from one publisher simultaneously
- Multiple publishers can write to the same topic

---

## Node-by-Node Breakdown

### HelloPublisher (`publisher.py`)

**Role:** sends a numbered text message every second.

```
Every 1.0 s
    │
    ▼
create String message: "Hello ROS2 #N"
    │
    ▼
publish → /hello_topic
    │
    ▼
log to terminal: "Publishing: Hello ROS2 #N"
```

Key details:
- Topic: `/hello_topic`
- Message type: `std_msgs/String`
- Timer interval: 1.0 s
- Uses a counter `self.count` to number each message

---

### HelloSubscriber (`subscriber.py`)

**Role:** listens on `/hello_topic` and prints every message it receives.

```
Message arrives on /hello_topic
    │
    ▼
callback(msg) is called by event loop
    │
    ▼
log to terminal: "Received: Hello ROS2 #N"
```

Key details:
- No timer — purely **event-driven**, only runs when a message arrives
- Identical `main()` pattern to the publisher (init → spin → shutdown)

---

### MarkerPublisher (`marker_publisher.py`)

**Role:** publishes a 3D cube every second so RViz2 can display it.

```
Every 1.0 s
    │
    ▼
build Marker object:
  - type: CUBE
  - position: (0, 0, 0.5) in "map" frame
  - size: 1×1×1 m
  - color: green (r=0, g=0.8, b=0.2, a=1.0)
  - lifetime: 2 s
    │
    ▼
publish → /visualization_marker
    │
    ▼
log to terminal: "Marker published at (0, 0, 0.5)"
```

Key details:
- Topic: `/visualization_marker`
- Message type: `visualization_msgs/Marker`
- `frame_id = 'map'` — tells RViz2 which coordinate system to use
- `marker.id = 0` + `marker.ns = 'hello_ros2'` — unique identity so updates overwrite instead of duplicate
- `marker.action = Marker.ADD` — create or update the marker
- `lifetime.sec = 2` — marker disappears from RViz2 automatically if the publisher stops

---

## How RViz2 Fits In

RViz2 is **not special** — it is just another ROS2 node that happens to have a 3D rendering window.

It subscribes to well-known topic names using the exact same API you use in your own nodes:

```python
# What RViz2 does internally when you add a Marker display:
self.create_subscription(Marker, '/visualization_marker', self.render_callback, 10)
```

### What RViz2 can visualize

RViz2 only understands **visual message types**. Plain `String` messages have no visual meaning and are not shown.

| Topic | Message type | What RViz2 shows |
|---|---|---|
| `/visualization_marker` | `visualization_msgs/Marker` | 3D shapes: cube, sphere, arrow, etc. |
| `/scan` | `sensor_msgs/LaserScan` | Lidar sweep as dots |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera feed panel |
| `/tf` | `tf2_msgs/TFMessage` | Coordinate frame axes |
| `/map` | `nav_msgs/OccupancyGrid` | 2D occupancy grid map |

### How to connect RViz2 to your marker

```
1. Run:  ros2 run hello_ros2 marker_publisher
2. Run:  rviz2
3. In RViz2 UI:
   - Set Fixed Frame → "map"
   - Click Add → By topic → /visualization_marker → Marker → OK
4. A green cube appears at (0, 0, 0.5) and refreshes every second
5. Stop the publisher → cube disappears after 2 seconds (lifetime)
```

---

## Event Loop

`rclpy.spin(node)` is the event loop — the heartbeat that keeps every node alive.

```
while node is alive:
    if timer fired       → call publish()
    if message arrived   → call callback(msg)
    otherwise            → sleep (near 0% CPU)
```

Without `spin()`, a node would register its publishers/subscriptions and immediately exit — nothing would ever be sent or received.

**Why not write your own `while True`?**

`spin()` handles:
- **Efficiency** — OS-level waiting, not busy-spinning
- **Fairness** — multiple timers and subscriptions are served in order
- **Thread safety** — callbacks run one at a time, no race conditions

You register callbacks (what to do), the event loop decides when to call them — this pattern is called **inversion of control**.

---

## Full System Flow

Putting it all together when all three programs run simultaneously:

```
t=0s
  MarkerPublisher starts  → announces /visualization_marker
  HelloPublisher starts   → announces /hello_topic
  HelloSubscriber starts  → subscribes to /hello_topic
  RViz2 starts            → subscribes to /visualization_marker

  DDS auto-connects:
    HelloPublisher      ↔ HelloSubscriber
    MarkerPublisher     ↔ RViz2

t=1s
  HelloPublisher timer fires
    → publishes "Hello ROS2 #0" on /hello_topic
    → HelloSubscriber callback fires: logs "Received: Hello ROS2 #0"

  MarkerPublisher timer fires
    → publishes green cube Marker on /visualization_marker
    → RViz2 render callback fires: draws cube at (0,0,0.5)

t=2s  (same pattern repeats)
  ...

t=Ns  (publisher stopped)
  No new Marker published
  After 2 seconds: cube disappears from RViz2 (lifetime expired)
```

---

## Summary

| | HelloPublisher | HelloSubscriber | MarkerPublisher | RViz2 |
|---|---|---|---|---|
| **Node name** | `hello_publisher` | `hello_subscriber` | `marker_publisher` | `rviz2` |
| **Role** | Publishes text | Receives text | Publishes 3D shape | Visualizes 3D data |
| **Topic** | `/hello_topic` (write) | `/hello_topic` (read) | `/visualization_marker` (write) | `/visualization_marker` (read) |
| **Driven by** | 1s timer | incoming messages | 1s timer | incoming messages |
| **Output** | Terminal log | Terminal log | Terminal log | 3D render window |
