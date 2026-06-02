# cyclonedds_macos.xml

Configuration file for **CycloneDDS** — the middleware ROS2 uses to send messages between nodes.
Loaded on macOS via the `CYCLONEDDS_URI` environment variable.

## Why macOS needs this file

macOS loopback (`lo0`) delivers multicast packets **3 times** (IPv4 + IPv6 + unicast paths).
Without this config, every ROS2 discovery message arrives triplicated → communication breaks.
Linux (Docker) does not have this problem and runs without any extra config.

## What it does — line by line

```xml
<NetworkInterface name="lo0" multicast="true" />
```
Lock DDS to loopback only (mirrors `ROS_LOCALHOST_ONLY=1`). Allow multicast on `lo0` so discovery still works.

```xml
<AllowMulticast>spdp</AllowMulticast>
```
Use multicast **only for discovery** (SPDP = node announcements). All topic data uses unicast.
This is the core fix — it prevents the 3x duplication on macOS loopback.

```xml
<ParticipantIndex>auto</ParticipantIndex>
```
Auto-assign a unique port index to each ROS2 process. Avoids port collisions when multiple tools
run simultaneously (`ros2 topic list`, Foxglove Studio, zenoh bridge, etc.).

## One-line summary

> Tell CycloneDDS: use localhost only, multicast for discovery only, auto-assign ports — fixes macOS loopback triple-delivery bug.
