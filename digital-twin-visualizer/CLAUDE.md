# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

- `npm run dev` — build with Rollup in watch mode and start `sirv` on `http://0.0.0.0:5055` with livereload.
- `npm run build` — one-off production build (minified via terser) to `public/build/bundle.js` + `bundle.css`.
- `npm run start` — serve the already-built `public/` on port 5055 (no rebuild).
- `npm run validate` — `svelte-check` type/template validation. There is no test runner, ESLint, or Prettier configured; do not invent a "run tests" step.

UI text and most comments are in Chinese — preserve that convention when editing.

## Architecture

Single-page Svelte 3 + TypeScript app bundled by Rollup (`rollup.config.js`, entry `src/main.ts` → `public/build/bundle.js`). Styling is Tailwind + DaisyUI compiled through `svelte-preprocess` + PostCSS inside the `<style global lang="postcss">` block in `src/Components/App/index.svelte`. No router — `App` toggles between two full-screen views via `activeTab`:

- **`Scene`** (`src/Components/Scene/`) — a three.js canvas showing a URDF robot loaded from `public/urdf/myRobot/urdf/robot.urdf` via `urdf-loader`. `createScene.ts` owns module-level singletons (`scene`, `camera`, `renderer`, `robot`, `controls`) and exports `loadRobot`, `rotateJoints`, `rotateRobotOnUpAxisChange`, and `updateDynamicPointClouds`. The drag-and-drop handler in `Scene/index.svelte` + `readDirectory.js` lets users drop a URDF folder; `loadMesh.ts` dispatches `.stl`/`.dae` via `loadSTL`/`loadDAE`.
- **`Monitor`** (`src/Components/Monitor/Monitor.svelte`) — dashboard with two MediaMTX iframes (`http://localhost:8889/cam1`, `/cam2`), a WebRTC `<video>` fed by `startWebRTC` (WHEP to `http://localhost:8889/cam3/whep`), process-parameter inputs, and two `CurrentCharter` Chart.js line charts for the two arms.

### Cross-component state (`src/stores/index.ts`)

Four Svelte writables glue the views together:
- `jointInfosStore: JointInfo[]` — populated by `Scene` after URDF load (filtered to exclude `base_jointA`/`base_jointB`), pushed live by `WSClient`, consumed by `JointControls` sliders and re-applied via the reactive `$: rotateJoints(jointInfos)` in `Scene/index.svelte`.
- `selectedUpAxisStore: string` — driven by `UpAxisDropdown`, handled by `rotateRobotOnUpAxisChange` (constants in `src/constants/axes.ts`).
- `isRobotConnected: boolean` — set to `true` after a successful `POST /connect`; `App` uses it as the reactive gate that lazily instantiates the two WebSocket clients.
- `lastPointCloudData` — persisted blob from the most recent `/weld` response so that `updateDynamicPointClouds` can reattach the point cloud after a URDF reload (see `manager.onLoad` in `createScene.ts`).

### Backend contract (all localhost)

The frontend assumes these services are running; there is no backend in this repo.

- `ws://127.0.0.1:8080` (`wsClient/wsClient.ts`) — joint telemetry, messages of shape `{ type: 'jointUpdate', data: JointInfo[] }`. Auto-reconnect with exponential backoff capped at 30 s.
- `ws://127.0.0.1:8081` (`wsClient/CurrentWSClient.ts`) — current telemetry, `{ type: 'CURRENT_DATA', armA: number[], armB: number[] }`; routed via callback to both `CurrentCharter` instances.
- `http://localhost:8082` — REST: `POST /connect` (connect arms), `POST /detect` with a JPEG body plus `X-Pipe-Diameter` / `X-Pipe-Thickness` headers (triggers point-cloud capture), `GET /weld` with `X-Current`, `X-Voltage`, `X-Speed`, `X-Frequency`, `X-Vibrate-Frequency`, `X-Stay-Time` headers (returns `{ cloudA, weldA, cloudB, weldB }` point clouds).
- `http://localhost:8889` — MediaMTX, used both for iframe HLS pages and for the WebRTC WHEP endpoint.

### Point-cloud coordinate convention

Raw point data from the backend is in millimetres and is divided by 1000 before being pushed into a `Float32BufferAttribute` (see `createGeometry` in `createScene.ts` and `loadPointCloud.ts`). Point clouds are added as children of the URDF links `base_frameA_link` / `base_frameB_link` so they inherit the link transform; if those links are missing the code falls back to the robot root and logs a warning. `loadPointCloud.ts` samples every 50th line (`count % 50 !== 0` skip) to keep the GPU budget down — adjust there if fidelity changes.

### Svelte reactivity gotchas specific to this codebase

- `createScene.ts` keeps `robot`, `scene`, etc. as module-scoped `let` bindings, not exports. Anything that needs them must go through the exported functions; don't try to import the variables directly.
- `updateDynamicPointClouds` is called from two places (`Monitor.showPointCloud` after the `/weld` fetch, and `manager.onLoad` after a URDF reload). Both paths must continue to read from `lastPointCloudData` to survive a view switch.
- The drawer sidebar uses Tailwind classes `translate-x-0` / `-translate-x-full` that are declared `:global` in `InterfaceDrawer.svelte` — do not remove the global block or the open/close animation breaks.
