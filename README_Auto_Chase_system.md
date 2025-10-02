# Auto Chase System (LiDAR Lock-On Follower)

**Auto Chase System** is a CARLA demo where **already-spawned police vehicles** **follow a LiDAR lock_on target** using **Pure Pursuit steering** and a **headway-based PI longitudinal controller**.
We integrated the raw data from the LiDAR mounted on the CARLA simulator vehicle into GPS coordinates through coordinate-frame transformations. As a result, we accurately obtained the target vehicle’s absolute map coordinates (x, y) detected by the LiDAR sensor and use these coordinates for vehicle following.

A single orchestrator (`./run_police_all.py`) launches and supervises both the detector and follower processes, handling retries and safe shutdown.

- Team: **A kiki**
- Key scripts:
  - Orchestrator: `run_police_all.py`
  - Target detector (headless): `auto_chase_detecter.py`
  - Follower controller: `auto_chase_lidar.py`


## Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [Components](#components)
- [Data Model & Topics](#data-model--topics)
- [Control Parameters](#control-parameters)
- [Requirements](#requirements)
- [Setup & Run](#setup--run)
- [Logs](#logs)
- [Tuning Guide](#tuning-guide)
- [Troubleshooting](#troubleshooting)
- [Repository Layout](#repository-layout)
- [FAQ](#faq)
- [License & Team](#license--team)



## Features

- **Auto start after vehicle detection**: When an external spawner brings up the vehicle, the orchestrator launches detector → follower.
- **LiDAR clustering for target selection**: Filters semantic *vehicle* points and clusters them; picks the nearest cluster as the target.
- **Lock-on gating**: Following is active only when `lock_on` is true; on false transition, emits cleanup/termination events.
- **Control**: **Pure Pursuit** for lateral; **headway-based PI** for longitudinal (with relative-speed term and safety caps).



## System Architecture

```
         ┌──────────────────────────┐
         │ External Spawner         │
         │ (hero spawn/map load)    │
         └─────────────┬────────────┘
                       │ CARLA RPC & Sensors
                ┌──────▼──────┐
                │ CARLA       │
                │ Server      │
                └──────┬──────┘
                       │ actors/sensors
     ┌─────────────────┼────────────────┐
     │                 │                │ 
┌────▼────────┐   ┌────▼────────────────┐
│ Detector    │   │ Follower Controller │
│ (headless)  │   │ auto_chase_lidar.py │
│ auto_chase_ │   └───────────┬─────────┘
│ detecter.py │               │ VehicleControl
└────┬────────┘               │
     │ LiDAR/Camera           │
     │ (ROS2/Zenoh)           │
     │                        │
     │ Zenoh Bus         ┌────▼─────────────────────────┐
     └──────────────────►│ subscribe: lock_on,target_gps│
                         │ publish: suspect/emergency   │
                         └──────────────────────────────┘
```

- **Detector**: Extracts vehicle points → clustering → selects target → publishes summary (no rendering/control; fully headless).
- **Follower**: Subscribes `lock_on`, `target_gps` and performs following (Pure Pursuit + headway PI).
- **Orchestrator**: Waits for hero → launches/monitors/restarts/terminates both processes; handles logging.

---

## Components

### 1) Orchestrator — `run_police_all.py`

- **Flow**
  1. Wait for hero vehicle (configurable timeout).
  2. Launch `auto_chase_detecter.py` → then `auto_chase_lidar.py`.
  3. If the detector exits too early, restart it up to *N* times.
  4. If either process ends, gracefully terminate both (SIGTERM→SIGKILL).

- **Environment (examples)**
  ```bash
  CARLA_HOST=localhost #you can change host PC
  CARLA_PORT=2000
  WAIT_HERO_TIMEOUT=40.0
  DETECTER_EARLY_WINDOW=10.0
  DETECTER_MAX_RETRY=2
  RUN_ALL_NICENESS=5
  POLL_INTERVAL=0.5
  ```

### 2) Target Detector (headless) — `auto_chase_detecter.py`

- **Purpose**: Aggregate semantic vehicle points, run lightweight radius clustering, and select the nearest cluster as target.
- **Input**: Semantic LiDAR(payload by zenoh), optional camera boxes.
- **Output**: Target summary (preferably with world coordinates) via Zenoh.
- **Notes**: No vehicle control or UI; minimized resource usage (“no-movement/headless”).

### 3) Follower Controller — `auto_chase_lidar.py`

- **Subscribe**
  - `lock_on`: "true"/"false".
  - `target_gps`: Recommended: targets[*].world = {x,y,z} (used directly). If world is missing, use gps={lat,lon} → ENU conversion (requires map geodetic origin; fallback via ORIGIN_LAT/LON env).

- **Control**
  - Lateral: Pure Pursuit with dynamic look-ahead based on speed/distance, clamped to [LA_MIN, LA_MAX].
  - Longitudinal: Headway PI + relative-speed term; throttle/brake caps; acceleration penalty at large steering angles.

---

## Data Model & Topics

**target_gps (recommended payload)**
```json
{
  "timestamp": 1727840000.12,
  "vehicle_id": "SUSPECT-42",
  "targets": [
    {
      "semantic_label": 14,
      "world": { "x": 123.4, "y": -56.7, "z": 0.5 }
    }
  ]
}
```

**lock_on**
```json
true
```

*Tip: If world exists it is used directly. If only gps is provided, ensure a valid map geodetic origin (or set ORIGIN_LAT/LON) for ENU conversion.*


## Control Parameters

*(override via environment variables)*
- `CONTROL_DT=0.05`
- `MAX_THROTTLE=0.85`
- `MAX_BRAKE=0.9`
- `MAX_STEER=1.0`
- `DESIRED_HEADWAY=1.6`
- `MIN_GAP=8.0`
- `CRUISE_V_MAX=18.0`
- `LOOKAHEAD_MIN=6.0`
- `LOOKAHEAD_MAX=22.0`
- `TARGET_TIMEOUT_S=1.2`
- `ZKEY_LOCKON=lock_on`
- `ZKEY_TARGET_GPS=target_gps`


## Requirements

- CARLA server (e.g., 0.9.15 / 0.9.16)
- Python 3.10+
- Python packages: carla, numpy, zenoh 


## Setup & Run

1. Start the CARLA server and spawn the hero vehicle (with your external tool/script).
2. (Optional) Start Zenoh and publish lock_on + target_gps from your perception pipeline.
3. Launch the orchestrator:
   ```bash
   ./run_police_all.py
   ```


## License & Team

- License: Fill in per project policy.
- Team: A kiki.
