# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

This is a **Visual Studio 2022** C++ project targeting **Windows x64**. Open `weld_backend.sln` in Visual Studio or build via MSBuild:

```bash
msbuild weld_backend.sln /p:Configuration=Release /p:Platform=x64
```

- Toolset: **v143**, C++ standard: **C++17**, character set: Unicode
- The project is a **Console Application** (`.exe`)

## Project Overview

A **dual-robot automated pipe welding backend**. It controls two robotic arms (HSC3 controllers at 192.168.1.71 and 192.168.1.72), each equipped with an FV laser profile sensor (192.168.1.61 and 192.168.1.62). The system scans pipe weld seams, detects weld points from point cloud data, and sends welding paths to the robots.

## Architecture

### Entry Point
- `main.cpp` — HTTP server (cpp-httplib) on **port 8082**, with 4 endpoints:
  - `POST /connect` — Connect to both robots and start WebSocket streaming servers
  - `POST /detect` — Receive a pipe end photo, compute distance via local Python service (port 8000), trigger dual-arm laser scan
  - `GET /weld` — Process accumulated point clouds, detect weld seams, return JSON with cloud/weld data for both arms
  - `GET /start` — Set arc welding wave parameters (width-based) and launch the welding program on both robots

### Key Components
- `robotConnect` — Thin wrapper over the HSC3 robot controller SDK (`CommApi`, `ProxyMotion`, `ProxyVar`, `ProxySys`, `ProxyVm`). Manages connection, motion commands, register I/O, and program load/unload.
- `LaserWorker` — Controls an FV laser profile sensor. Reads robot pose + timestamp at each scan frame, transforms laser points from sensor frame → robot base frame via calibration matrix + robot orientation matrix, accumulates into `pcl::PointCloud` and `std::vector<TimestampedPoint>`. Uses a ring buffer (`RobotInfo[3000]`) for time synchronization between robot pose and laser data.
- `WeldRecongnition` — Core weld seam detection pipeline:
  1. PCL StatisticalOutlierRemoval (different params per arm: A=tight, B=loose)
  2. `solveWeldByPCA`: groups points by timestamp, runs PCA per group, fits a parabola via SVD, finds curvature extrema (left/right "corners" of the weld groove), computes weighted centroids and weld width
  3. `filterOverlappingPoints` — Deduplicates overlapping welds between the two arms
  4. `fitCircle3D` + `offsetAlongBendRadius` — Fit a 3D circle to weld points, offset along radial direction
  5. `fitAndResamplePath` — Arc-length resampling to a fixed number of points (default 20)
- `PointCloudProcessor` — Statistical outlier removal, timestamp grouping, RDP polyline simplification, perpendicular distance to line segment
- `Utils` — 3D vector math, trapezoid fitting (for weld groove cross-section), half-circle pose computation for scan trajectories
- `Calculate` — HTTP client (Boost.Beast) that sends an image to `127.0.0.1:8000/detect-distance` (Python service) and parses the returned `distance_m` value
- `DigitalTwinServer` — WebSocket server (Boost.Beast) on **port 8080**, streams joint angles for a 3D digital twin frontend. Hardcoded joint names map to a dual-arm robot model.
- `CurrentServerWorker` — WebSocket server on **port 8081**, streams motor current/sensor data from both robots.

### Data Flow
```
Photo → Calculate::getDistance() → pipe distance
  ↓
POST /detect → load SCAN_1.PRG on robots → LaserWorker::asyncStart()
  ↓
LaserWorker collects TimestampedPoint clouds (laser → robot frame transform)
  ↓
GET /weld → WeldRecongnition::loadAndProcess() → PCA + curve fitting → weld points
  ↓
fitAndResamplePath(20 points) → write to robot LR registers via setLR()
  ↓
POST /start → set arc wave width → load RUN.PRG → start welding
```

### External Dependencies (in `external/`)
- **PCL 1.12.1** — Point cloud processing (filters, kdtree, I/O)
- **Open3D 0.19.0** — Included but not heavily used in current code
- **HSC3 API** — Robot controller communication SDK
- **FV Laser SDK 1.6.4** — Laser profile sensor SDK (`fvlaser.lib`, `fvbase.lib`, `xtr.lib`)
- **Boost 1.78** — ASIO, Beast (WebSocket/HTTP)
- **Eigen** — Linear algebra (PCA, SVD, circle fitting)
- **cpp-httplib** (header-only, `httplib.h`) — HTTP server
