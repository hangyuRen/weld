<script lang="ts">
  import { onDestroy, onMount } from 'svelte';
  import * as THREE from 'three';
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

  export let title: string = '点云';
  // 是否折叠（支持父组件 bind:collapsed 双向绑定）
  export let collapsed: boolean = false;

  // ====== DOM refs ======
  let viewerEl: HTMLDivElement;
  let tooltipEl: HTMLDivElement;
  let loadingEl: HTMLDivElement;

  // ====== UI state ======
  let statusText = '请先上传文件...';
  let loading = false;

  // ====== Three.js objects ======
  let scene: THREE.Scene;
  let camera: THREE.PerspectiveCamera;
  let renderer: THREE.WebGLRenderer;
  let controls: OrbitControls;

  let pointsMesh: THREE.Points<THREE.BufferGeometry, THREE.ShaderMaterial> | null = null;

  let raycaster: THREE.Raycaster;
  let mouse: THREE.Vector2;

  let solidPointMaterial: THREE.ShaderMaterial;

  let rafId = 0;
  let resizeObserver: ResizeObserver | null = null;

  // ====== data ======
  type P3 = { x: number; y: number; z: number };
  type PointState = { type: 'feature' | 'normal'; isSelected: boolean };

  let rawCloudData: P3[] = [];
  let featurePoints: P3[] = [];
  let pointStates: PointState[] = [];

  // ====== config (from your HTML) ======
  const COLOR_NORMAL = new THREE.Color(0xaaaaaa);
  const COLOR_FEATURE = new THREE.Color(0x00ff00);
  const COLOR_SELECTED = new THREE.Color(0xff0000);
  const BASE_SIZE = 7.0;

  function getSize() {
    const rect = viewerEl.getBoundingClientRect();
    const width = Math.max(1, Math.floor(rect.width));
    const height = Math.max(1, Math.floor(rect.height));
    return { width, height };
  }

  function init() {
    // scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a1a);

    const { width, height } = getSize();

    // camera
    camera = new THREE.PerspectiveCamera(60, width / height, 1, 50000);
    camera.position.set(0, 100, 1500);

    // renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(width, height);
    viewerEl.appendChild(renderer.domElement);

    // controls
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;

    // raycaster
    raycaster = new THREE.Raycaster();
    raycaster.params.Points.threshold = 0.5;
    mouse = new THREE.Vector2();

    // shader material (your solid circle point shader)
    solidPointMaterial = new THREE.ShaderMaterial({
      uniforms: {
        scale: { value: height / 2.0 },
        baseSize: { value: BASE_SIZE }
      },
      vertexShader: `
        attribute vec3 color;
        varying vec3 vColor;
        uniform float scale;
        uniform float baseSize;

        void main() {
          vColor = color;
          vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );

          gl_PointSize = baseSize * ( scale / -mvPosition.z );
          gl_PointSize = max(gl_PointSize, 3.0);

          gl_Position = projectionMatrix * mvPosition;
        }
      `,
      fragmentShader: `
        varying vec3 vColor;

        void main() {
          float dist = length(gl_PointCoord - vec2(0.5, 0.5));
          float delta = fwidth(dist);
          float alpha = 1.0 - smoothstep(0.5 - delta, 0.5 + delta, dist);

          if (alpha < 0.01) discard;

          gl_FragColor = vec4(vColor, alpha);
        }
      `,
      transparent: true,
      depthTest: true,
      depthWrite: true,
      blending: THREE.NormalBlending
    });

    // prevent events leaking to background scene (if background uses window handlers)
    // Important: add AFTER OrbitControls so controls still work.
    const stop = (e: Event) => e.stopPropagation();
    renderer.domElement.addEventListener('pointerdown', stop);
    renderer.domElement.addEventListener('pointerup', stop);
    renderer.domElement.addEventListener('pointermove', stop);
    renderer.domElement.addEventListener('wheel', stop, { passive: true });
    renderer.domElement.addEventListener('contextmenu', stop);

    // our interaction listeners (bound to canvas only)
    renderer.domElement.addEventListener('pointermove', onPointerMove);
    renderer.domElement.addEventListener('pointerleave', () => hideTooltip());
    renderer.domElement.addEventListener('click', onClick);
  }

  function onResize() {
    if (!renderer || !camera || !solidPointMaterial) return;

    const { width, height } = getSize();
    camera.aspect = width / height;
    camera.updateProjectionMatrix();

    renderer.setSize(width, height);
    if (solidPointMaterial.uniforms?.scale) {
      solidPointMaterial.uniforms.scale.value = height / 2.0;
    }
  }

  function toggleCollapsed(e?: MouseEvent) {
    // 仅作用于面板本身，避免冒泡影响底层 three 场景
    if (e) e.stopPropagation();
    collapsed = !collapsed;

    // 展开后下一帧触发一次 resize，保证 canvas 正确填充
    if (!collapsed) {
      setTimeout(() => onResize(), 0);
    }
  }

  function animate() {
    rafId = requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }

  async function handleFile(event: Event, type: 'cloud' | 'feature') {
    const input = event.currentTarget as HTMLInputElement;
    const file = input.files?.[0];
    if (!file) return;

    loading = true;
    if (loadingEl) loadingEl.style.display = 'block';

    // let UI paint the loading overlay
    await new Promise((r) => setTimeout(r, 50));

    const text = await file.text();
    if (type === 'cloud') parseCloud(text);
    else parseFeatures(text);

    renderAll();

    loading = false;
    if (loadingEl) loadingEl.style.display = 'none';
  }

  function parseCloud(text: string) {
    rawCloudData = [];
    const lines = text.split('\n');
    let count = 0;
    for (let line of lines) {
      if (count++ % 2 == 0) continue;
      line = line.trim();
      if (!line) continue;
      const parts = line.split(/[,\s]+/);
      if (parts.length >= 3) {
        const x = parseFloat(parts[0]);
        const y = parseFloat(parts[1]);
        const z = parseFloat(parts[2]);
        if (!Number.isNaN(x) && !Number.isNaN(y) && !Number.isNaN(z)) {
          rawCloudData.push({ x, y, z });
        }
      }
    }
    updateStatus();
  }

  function parseFeatures(text: string) {
    featurePoints = [];
    const lines = text.split('\n');

    let startIndex = 0;
    if (lines[0]) {
      const first = lines[0].trim();
      if (first) {
        const p0 = first.split(/[,\s]+/)[0];
        if (Number.isNaN(parseFloat(p0))) startIndex = 1; // skip header
      }
    }

    for (let i = startIndex; i < lines.length; i++) {
      const line = lines[i].trim();
      if (!line) continue;
      const p = line.split(/[,\s]+/);

      // compatible with your 8-column format
      if (p.length >= 8) {
        const lx = parseFloat(p[2]);
        const ly = parseFloat(p[3]);
        const lz = parseFloat(p[4]);
        const rx = parseFloat(p[5]);
        const ry = parseFloat(p[6]);
        const rz = parseFloat(p[7]);

        if (!Number.isNaN(lx) && !Number.isNaN(ly) && !Number.isNaN(lz))
          featurePoints.push({ x: lx, y: ly, z: lz });
        if (!Number.isNaN(rx) && !Number.isNaN(ry) && !Number.isNaN(rz))
          featurePoints.push({ x: rx, y: ry, z: rz });
      } else if (p.length >= 3) {
        const x = parseFloat(p[0]);
        const y = parseFloat(p[1]);
        const z = parseFloat(p[2]);
        if (!Number.isNaN(x) && !Number.isNaN(y) && !Number.isNaN(z)) {
          featurePoints.push({ x, y, z });
        }
      }
    }

    updateStatus();
  }

  function updateStatus() {
    statusText = `点云: ${rawCloudData.length} | 特征点: ${featurePoints.length}`;
  }

  function renderAll() {
    if (!scene || (!rawCloudData.length && !featurePoints.length)) return;

    if (pointsMesh) {
      scene.remove(pointsMesh);
      pointsMesh.geometry.dispose();
      // material is shared (solidPointMaterial), do not dispose here
      pointsMesh = null;
    }

    const positions: number[] = [];
    const colors: number[] = [];
    pointStates = [];

    const featureSet = new Set<string>();
    for (const p of featurePoints) {
      featureSet.add(`${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`);
    }

    // feature points first
    for (const p of featurePoints) {
      positions.push(p.x, p.y, p.z);
      colors.push(COLOR_FEATURE.r, COLOR_FEATURE.g, COLOR_FEATURE.b);
      pointStates.push({ type: 'feature', isSelected: false });
    }

    // normal points (excluding feature duplicates)
    for (const p of rawCloudData) {
      const key = `${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`;
      if (featureSet.has(key)) continue;

      positions.push(p.x, p.y, p.z);
      colors.push(COLOR_NORMAL.r, COLOR_NORMAL.g, COLOR_NORMAL.b);
      pointStates.push({ type: 'normal', isSelected: false });
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    geometry.computeBoundingSphere();

    pointsMesh = new THREE.Points(geometry, solidPointMaterial);
    scene.add(pointsMesh);

    const bs = geometry.boundingSphere;
    if (!bs) return;

    const center = bs.center.clone();
    const radius = bs.radius || 100;

    controls.target.copy(center);

    const distance = radius * 2.5;
    if (radius > 500) {
      camera.position.set(center.x, center.y + distance * 0.5, center.z + distance);
    } else {
      camera.position.set(center.x, center.y, center.z + distance);
    }

    camera.near = Math.max(0.1, radius / 100);
    camera.far = Math.max(2000, radius * 50);
    camera.updateProjectionMatrix();
  }

  function toNDC(event: PointerEvent | MouseEvent) {
    const rect = renderer.domElement.getBoundingClientRect();
    const x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    const y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    return { x, y, rect };
  }

  function hideTooltip() {
    if (!tooltipEl) return;
    tooltipEl.style.display = 'none';
  }

  function onPointerMove(event: PointerEvent) {
    if (!pointsMesh) return;

    const { x, y, rect } = toNDC(event);
    mouse.x = x;
    mouse.y = y;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObject(pointsMesh);

    if (!intersects.length) {
      hideTooltip();
      renderer.domElement.style.cursor = 'default';
      return;
    }

    const index = (intersects[0] as any).index as number;
    const pos = pointsMesh.geometry.attributes.position.array as unknown as number[];
    const state = pointStates[index];

    let statusHtml = '';
    if (state.isSelected) statusHtml = '<span class="tag red">● 已焊接 (Welded)</span>';
    else if (state.type === 'feature') statusHtml = '<span class="tag green">● 特征点 (Feature)</span>';
    else statusHtml = '<span class="tag normal">● 普通点 (Normal)</span>';

    tooltipEl.innerHTML = `
      <div class="coord">X: ${pos[index * 3].toFixed(3)}</div>
      <div class="coord">Y: ${pos[index * 3 + 1].toFixed(3)}</div>
      <div class="coord">Z: ${pos[index * 3 + 2].toFixed(3)}</div>
      ${statusHtml}
    `;

    const localX = event.clientX - rect.left + 12;
    const localY = event.clientY - rect.top + 12;
    tooltipEl.style.left = `${localX}px`;
    tooltipEl.style.top = `${localY}px`;
    tooltipEl.style.display = 'block';

    renderer.domElement.style.cursor = 'pointer';
  }

  function onClick(event: MouseEvent) {
    if (!pointsMesh) return;

    const { x, y } = toNDC(event);
    mouse.x = x;
    mouse.y = y;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObject(pointsMesh);

    if (!intersects.length) return;

    const index = (intersects[0] as any).index as number;
    const colorAttr = pointsMesh.geometry.attributes.color as THREE.BufferAttribute;
    const state = pointStates[index];

    if (state.isSelected) {
      const restore = state.type === 'feature' ? COLOR_FEATURE : COLOR_NORMAL;
      colorAttr.setXYZ(index, restore.r, restore.g, restore.b);
      state.isSelected = false;
    } else {
      colorAttr.setXYZ(index, COLOR_SELECTED.r, COLOR_SELECTED.g, COLOR_SELECTED.b);
      state.isSelected = true;
    }

    colorAttr.needsUpdate = true;
  }

  onMount(() => {
    init();
    onResize();
    animate();

    resizeObserver = new ResizeObserver(() => onResize());
    resizeObserver.observe(viewerEl);

    return () => {
      // nothing
    };
  });

  onDestroy(() => {
    cancelAnimationFrame(rafId);

    if (resizeObserver && viewerEl) {
      resizeObserver.unobserve(viewerEl);
      resizeObserver.disconnect();
    }

    if (pointsMesh) {
      scene.remove(pointsMesh);
      pointsMesh.geometry.dispose();
      pointsMesh = null;
    }

    if (solidPointMaterial) solidPointMaterial.dispose();

    if (controls) controls.dispose();

    if (renderer) {
      renderer.domElement.removeEventListener('pointermove', onPointerMove);
      renderer.domElement.removeEventListener('click', onClick);
      renderer.dispose();
      // remove canvas
      const canvas = renderer.domElement;
      if (canvas && canvas.parentElement) canvas.parentElement.removeChild(canvas);
    }
  });
</script>

<div class="pc-panel" class:collapsed>
  <div class="pc-header">
    <div class="pc-title">{title}</div>
    <button class="pc-collapse" on:click={toggleCollapsed} aria-label={collapsed ? '展开' : '折叠'}>
      {#if collapsed}
        展开
      {:else}
        折叠
      {/if}
    </button>
  </div>

  <div class="pc-toolbar">
    <div class="input-group">
      <label>点云文件 (.txt)</label>
      <input type="file" accept=".txt" on:change={(e) => handleFile(e, 'cloud')} />
    </div>

    <div class="input-group">
      <label>特征点文件 (.csv / .txt)</label>
      <input type="file" accept=".csv,.txt" on:change={(e) => handleFile(e, 'feature')} />
    </div>

    <div class="legend">
      <span class="dot normal"></span><span class="txt">普通</span>
      <span class="dot feature"></span><span class="txt">特征</span>
      <span class="dot selected"></span><span class="txt">已焊接</span>
    </div>

    <div class="status">{statusText}</div>
  </div>

  <div class="pc-viewer" bind:this={viewerEl}>
    <div class="loading" bind:this={loadingEl} style="display:none;">正在处理数据，请稍候...</div>
    <div class="tooltip" bind:this={tooltipEl}></div>
  </div>
</div>

<style>
  .pc-panel {
    width: 100%;
    height: 100%;
    display: grid;
    grid-template-rows: 34px auto 1fr;
    border-radius: 10px;
    overflow: hidden;
    border: 1px solid rgba(255,255,255,0.12);
    background: rgba(20, 20, 20, 0.75);
    backdrop-filter: blur(6px);
  }

  .pc-header {
    display: flex;
    align-items: center;
    padding: 0 12px;
    font-size: 13px;
    color: #fff;
    background: rgba(40, 40, 40, 0.85);
    border-bottom: 1px solid rgba(255,255,255,0.08);
    user-select: none;
  }

  .pc-title {
    font-size: 13px;
    color: #fff;
  }

  .pc-collapse {
    margin-left: auto;
    height: 22px;
    padding: 0 10px;
    border-radius: 999px;
    font-size: 12px;
    color: #ddd;
    background: rgba(255,255,255,0.08);
    border: 1px solid rgba(255,255,255,0.12);
    cursor: pointer;
  }
  .pc-collapse:hover {
    background: rgba(255,255,255,0.12);
  }

  /* 折叠态：只保留标题栏，不占用大块遮挡区域 */
  .pc-panel.collapsed {
    grid-template-rows: 34px 0 0;
  }
  .pc-panel.collapsed .pc-toolbar,
  .pc-panel.collapsed .pc-viewer {
    display: none;
  }

  .pc-toolbar {
    display: flex;
    flex-wrap: wrap;
    align-items: flex-end;
    gap: 10px;
    padding: 8px 12px;
    background: rgba(25, 25, 25, 0.65);
    border-bottom: 1px solid rgba(255,255,255,0.06);
  }

  .input-group {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  label {
    font-size: 12px;
    color: #aaa;
    user-select: none;
  }

  input[type="file"] {
    width: 180px;
    font-size: 12px;
    color: #ddd;
  }

  .legend {
    display: flex;
    align-items: center;
    gap: 6px;
    margin-left: 4px;
  }

  .dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    display: inline-block;
    border: 1px solid rgba(0,0,0,0.3);
  }
  .dot.normal { background: #aaaaaa; }
  .dot.feature { background: #00ff00; }
  .dot.selected { background: #ff0000; }

  .txt {
    font-size: 12px;
    color: #ccc;
    margin-right: 6px;
    user-select: none;
  }

  .status {
    margin-left: auto;
    font-size: 12px;
    color: #55ff55;
    user-select: none;
    white-space: nowrap;
  }

  .pc-viewer {
    position: relative;
    width: 100%;
    height: 100%;
    background: #1a1a1a;
  }

  /* make the appended canvas fill the container */
  .pc-viewer :global(canvas) {
    display: block;
    width: 100%;
    height: 100%;
  }

  .loading {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    padding: 10px 16px;
    border-radius: 8px;
    background: rgba(0,0,0,0.85);
    color: #fff;
    z-index: 20;
    user-select: none;
    font-size: 12px;
  }

  .tooltip {
    position: absolute;
    display: none;
    background: rgba(30, 30, 30, 0.95);
    border: 1px solid #666;
    color: #fff;
    padding: 8px 10px;
    border-radius: 6px;
    font-size: 12px;
    pointer-events: none;
    z-index: 30;
    white-space: nowrap;
    box-shadow: 0 2px 5px rgba(0,0,0,0.4);
  }

  .tooltip :global(.coord) {
    color: #ccc;
    margin-bottom: 4px;
    font-family: monospace;
  }

  .tooltip :global(.tag) {
    font-weight: bold;
    display: block;
    margin-top: 5px;
  }
  .tooltip :global(.tag.red) { color: #ff5555; }
  .tooltip :global(.tag.green) { color: #55ff55; }
  .tooltip :global(.tag.normal) { color: #cccccc; }
</style>
