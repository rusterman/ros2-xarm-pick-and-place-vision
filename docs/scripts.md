# Scripts Reference

All scripts live in `scripts/` and exist because ROS2 nodes run inside Docker while the macOS host needs its own tools (CLI, Foxglove Studio) to interact with them. Each script handles a specific piece of that setup.

---

## Execution order

```
1. download_zenoh.sh   # once, after cloning
2. docker compose up -d
3. start_bridge.sh     # keep running in a dedicated terminal
4. ros2.sh             # use freely in other terminals
```

---

## download_zenoh.sh

**Run once after cloning.**

Downloads the `zenoh-bridge-ros2dds` binary from GitHub releases into `bin/`. Detects CPU arch automatically (Apple Silicon → `aarch64`, Intel → `x86_64`) and fetches the matching build. All other scripts depend on this binary existing.

---

## start_bridge.sh

**The core link between Docker and macOS. Must stay running.**

Launches the Zenoh bridge in two halves:

| Side | Role | What it does |
|------|------|--------------|
| Docker (router) | Server | Listens on port 7447, exposes ROS2 topics from inside the container |
| macOS (client) | Consumer | Connects to `localhost:7447` via Docker's port-forward, receives those topics |

Both sides use `ROS_DOMAIN_ID=42` and `--no-multicast-scouting` (TCP-only, no UDP broadcast). Without this script running, `ros2.sh` and Foxglove Studio see no topics from Docker.

---

## ros2.sh

**Wrapper for any `ros2 ...` CLI command on macOS.**

Usage:
```bash
./scripts/ros2.sh topic list
./scripts/ros2.sh topic echo /hello_topic
./scripts/ros2.sh node list
```

Fixes three macOS-specific problems:

1. **Homebrew Python pollution** — clears `PYTHONPATH` so Homebrew's Python 3.14+ doesn't break rclpy's C extension loading.
2. **Daemon hang** — passes `--no-daemon` to graph-query verbs (`topic list`, `node list`, etc.) so they don't wait 8–60 s for the ROS2 daemon. Streaming verbs (`hz`, `bw`, `pub`) don't get this flag — they never use the daemon.
3. **DDS interface** — sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and points CycloneDDS at the loopback interface (`lo0`) via `config/cyclonedds_macos.xml`, matching what the Zenoh bridge uses.

---

## Shared configuration

All scripts that talk to ROS2 on macOS use the same three settings:

| Setting | Value | Purpose |
|---------|-------|---------|
| `ROS_DOMAIN_ID` | `42` | Isolates this project's traffic |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | Consistent DDS layer |
| `CYCLONEDDS_URI` | `config/cyclonedds_macos.xml` | Forces loopback interface (`lo0`) |
