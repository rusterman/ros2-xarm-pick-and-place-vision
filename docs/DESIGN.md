# Design Notes — ROS2 on macOS via Docker

## The core problem

ROS2 nodes communicate using **DDS** (Data Distribution Service) — a pub/sub protocol that auto-discovers peers by broadcasting UDP packets over the local network (multicast).

Docker Desktop on macOS does **not** run containers on the host network. Containers live inside a hidden Linux VM with their own virtual network. macOS cannot receive DDS broadcast traffic from that VM, so:

- `ros2 topic list` from macOS sees nothing
- Foxglove Studio sees no topics
- Nodes inside Docker and tools on macOS are completely invisible to each other

## Why the obvious fixes don't work

| Approach | Why it fails on macOS |
|---|---|
| `--network=host` | "Host" means the Linux VM, not macOS. Mac tools still can't reach it. |
| CycloneDDS unicast peers | Requires hardcoded container IPs that change on every restart; fragile. |
| FastDDS Discovery Server | More complex setup for the same result; less tooling support. |
| Native ROS2 on macOS | Experimental, poorly supported, breaks frequently. Not viable for real work. |

## Why Zenoh

`zenoh-bridge-ros2dds` solves this by acting as a translator between two worlds:

- It speaks **DDS** on each side (so ROS2 nodes see normal topics)
- It relays messages over **plain TCP** between the two sides

TCP works fine across Docker's port-forward (`7447:7447`), which is exactly what Docker Desktop does support on macOS.

The bridge is officially maintained by Eclipse and increasingly integrated into the ROS2 ecosystem — it is not a workaround, it is the intended solution for this architecture.

## How the bridge works

```
macOS
  Foxglove Studio / ros2 CLI
       ↕ DDS (loopback)
  zenoh-bridge [client]          ← bin/zenoh-bridge-ros2dds (macOS binary)
       ↕ TCP localhost:7447
─────────────────────────────── Docker port-forward
       ↕ TCP 0.0.0.0:7447
  zenoh-bridge [router]          ← installed inside ros2_dev container
       ↕ DDS (domain 42)
  ROS2 nodes (publisher, subscriber, marker_publisher)
```

Two bridge instances run simultaneously:

- **Router** (inside Docker) — listens on TCP port 7447, bridges to ROS2 nodes via DDS
- **Client** (on macOS) — connects to `localhost:7447` via Docker's port-forward, bridges to macOS tools via DDS

From any ROS2 tool's perspective (on either side), all topics look local. The TCP relay is invisible.

## Why a binary in `bin/`

The macOS-side bridge must run **on macOS**, not inside Docker. It is a native macOS binary (not a Python package, not in any package manager). The `bin/` folder stores it after `scripts/download_zenoh.sh` fetches it. It is gitignored because it is a large platform-specific binary that can always be re-downloaded.

## CycloneDDS config (`config/cyclonedds_macos.xml`)

Without this config, macOS receives each DDS message three times (via IPv4, IPv6, and multicast paths on the loopback interface). The config sets `AllowMulticast=spdp` — discovery uses multicast, but data delivery uses unicast only — eliminating the triplication.
