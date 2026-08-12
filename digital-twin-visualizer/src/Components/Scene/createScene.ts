import {
  Scene,
  PerspectiveCamera,
  WebGLRenderer,
  DirectionalLight,
  AmbientLight,
  PCFSoftShadowMap,
  Object3D,
  Box3,
  Vector3,
  MathUtils,
  LoadingManager,
} from 'three';

import * as THREE from "three";
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader, { URDFRobot } from 'urdf-loader';
import { get } from 'svelte/store';
import { jointInfosStore, selectedUpAxisStore, lastPointCloudData, pipeDiameterStore } from '../../stores';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import type { JointInfo } from '../../types';
import getFileNameFromPath from './utils/getFileNameFromPath';
import scaleInView from './utils/scaleInView';
import { loadSTL, loadDAE } from './utils/loadMesh';
import setRobotRotation from './utils/setRobotRotation';
import * as axes from '../../constants/axes';
import { loadPointCloudA, loadPointCloudB, scale } from './loadPointCloud';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';

// 定义全局变量
let transformControls: TransformControls;
let baseFrameAnchor: THREE.Object3D;

const URDF_FILE_PATH = '../urdf/myRobot/urdf/robot.urdf';
// const POINT_CLOUD_URLA = '../pointcloud/pointcloudA1770200071.txt';
// const POINT_CLOUD_URLB = '../pointcloud/pointcloudB1770200071.txt';
let dynamicPointsA: THREE.Points | null = null;
let dynamicPointsB: THREE.Points | null = null;
let dynamicWeldA: THREE.Line | null = null;
let dynamicWeldB: THREE.Line | null = null;
let pipeMesh: THREE.Mesh | null = null;
let pipeGroup: THREE.Group | null = null;
let pipeDiamUnsub: (() => void) | null = null;

/*

THREE.js
   Y
   |
   |
   .-----X
 ／
Z

ROS URDf
       Z
       |   X
       | ／
 Y-----.

*/

let scene: Scene;
let camera: PerspectiveCamera;
let renderer: WebGLRenderer;
let manager: LoadingManager;
let loader: URDFLoader;
let robot: URDFRobot;
let controls: OrbitControls;
let box: Box3;

export function updateDynamicPointClouds(data: any): void {
  if (!robot) {
    lastPointCloudData.set(data);
    return;
  }

  // 1. 获取目标挂载点 (基准坐标系)
  const targetFrameA = robot.getObjectByName('base_frameA_link') || robot;
  const targetFrameB = robot.getObjectByName('base_frameB_link') || robot;

  // 清理旧的对象
  const cleanup = (obj: THREE.Object3D | null, parent: THREE.Object3D) => {
    if (obj) parent.remove(obj);
  };
  cleanup(dynamicPointsA, targetFrameA);
  cleanup(dynamicWeldA, targetFrameA);
  cleanup(dynamicPointsB, targetFrameB);
  cleanup(dynamicWeldB, targetFrameB);

  // 2. 创建并挂载 A 组 (白色)
  dynamicPointsA = createGeometry(data.cloudA, 'Points', 0xffffff);
  dynamicWeldA = createGeometry(data.weldA, 'Line', 0xff0000);
  targetFrameA.add(dynamicPointsA);
  targetFrameA.add(dynamicWeldA);

  // 3. 创建并挂载 B 组 (蓝色)
  dynamicPointsB = createGeometry(data.cloudB, 'Points', 0x00aaff);
  dynamicWeldB = createGeometry(data.weldB, 'Line', 0x00ff00);
  targetFrameB.add(dynamicPointsB);
  targetFrameB.add(dynamicWeldB);

  console.log(`点云已挂载至 A:[${targetFrameA.name}] 和 B:[${targetFrameB.name}]`);
}

/**
 * 通用几何体创建工厂
 */
function createGeometry(pts: any[], type: 'Points' | 'Line', color: number): any {
  const positions: number[] = [];

  pts.forEach((p, index) => {
    // 这里的抽样逻辑 (如 % 10) 可以根据性能需要添加
    const x = parseFloat(p.x) / 1000;
    const y = parseFloat(p.y) / 1000;
    const z = parseFloat(p.z) / 1000;

    if (!isNaN(x) && !isNaN(y) && !isNaN(z)) {
      // 保持和你之前代码一致的坐标映射
      positions.push(x, y, z);
    }
  });

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));

  if (type === 'Points') {
    const material = new THREE.PointsMaterial({ size: 0.002, color: color });
    const points = new THREE.Points(geometry, material);
    // 重置局部变换，确保相对于 targetFrame 的偏移为 0
    points.position.set(0, 0, 0);
    return points;
  } else {
    const material = new THREE.LineBasicMaterial({ color: color, linewidth: 2 });
    const line = new THREE.Line(geometry, material);
    line.position.set(0, 0, 0);
    return line;
  }
}


function createScene(canvasEl: HTMLCanvasElement): void {
  init(canvasEl);
  render();
}

function init(canvasEl: HTMLCanvasElement): void {
  // *** Initialize three.js scene ***

  scene = new Scene();

  const fov = 45;
  const aspectRatio = window.innerWidth / window.innerHeight;
  const near = 0.1;
  const far = 100;
  camera = new PerspectiveCamera(fov, aspectRatio, near, far);
  camera.position.set(2, 2, 2);

  renderer = new WebGLRenderer({ antialias: true, canvas: canvasEl });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = PCFSoftShadowMap;
  renderer.localClippingEnabled = true;

  // Lighting
  var lights = [];
  lights[0] = new THREE.PointLight(0x304ffe, 1, 0);
  lights[1] = new THREE.PointLight(0xffffff, 1, 0);
  lights[2] = new THREE.PointLight(0xffffff, 1, 0);
  lights[0].position.set(0, 200, 0);
  lights[1].position.set(100, 200, 100);
  lights[2].position.set(-100, -200, -100);
  scene.add(lights[0]);
  scene.add(lights[1]);
  scene.add(lights[2]);

  // Background
  scene.background = new THREE.Color(0x2A303C); //0x263238

  // Grid
  // const size = 100;
  // const divisions = 100;
  // const gridHelper = new THREE.GridHelper( size, divisions );
  // scene.add( gridHelper );

  // Axes Helper
  // var helper = new THREE.AxesHelper(2);
  // var colors = helper.geometry.attributes.color;
  // colors.setXYZ(0, 1, 0, 0);
  // colors.setXYZ(1, 1, 0, 0);
  // colors.setXYZ(2, 0, 0, 1);
  // colors.setXYZ(3, 0, 0, 1);
  // colors.setXYZ(4, 0, 1, 0);
  // colors.setXYZ(5, 0, 1, 0);
  // scene.add(helper);

  // Allow user to rotate around the robot.
  controls = new OrbitControls(camera, renderer.domElement);
  controls.minDistance = 1;

  // *** Load URDF ***

  manager = new LoadingManager();
  loader = new URDFLoader(manager);

  controls = new OrbitControls(camera, renderer.domElement);

  // 2. 调用你的控制函数
  // initTransformControls();

  loadRobot();

  pipeDiamUnsub = pipeDiameterStore.subscribe((diameter: number) => {
    if (pipeMesh) {
      const diamScale = (diameter / 300) / 1000;
      pipeMesh.scale.set(diamScale, diamScale, diamScale);
    }
  });

  // *** Resize the contents of the canvas on window resize.

  window.addEventListener('resize', onResize);
}


function initTransformControls() {
  // 1. 创建一个空物体作为“基坐标系”
  // 虽然是“空”的，但为了可见，我们可以放一个小的 AxesHelper
  baseFrameAnchor = new THREE.Object3D();
  const axesHelper = new THREE.AxesHelper(0.5); // 0.5米长的轴
  baseFrameAnchor.add(axesHelper);
  scene.add(baseFrameAnchor);

  // 2. 初始化变换控制器
  transformControls = new TransformControls(camera, renderer.domElement);

  // 重点：当拖拽物体时，必须禁用 OrbitControls，否则两者会冲突
  transformControls.addEventListener('dragging-changed', (event) => {
    controls.enabled = !event.value;
  });

  // 3. 监听变化并打印坐标/旋转
  transformControls.addEventListener('change', () => {
    const pos = baseFrameAnchor.position;
    const rot = baseFrameAnchor.rotation;

    // 你可以在这里更新 UI 或 Stores
    console.log(`基坐标系位置: x:${pos.x.toFixed(3)}, y:${pos.y.toFixed(3)}, z:${pos.z.toFixed(3)}`);
    console.log(`基坐标系旋转 (Rad): x:${rot.x.toFixed(3)}, y:${rot.y.toFixed(3)}, z:${rot.z.toFixed(3)}`);
  });

  // 4. 将控制器绑定到锚点上
  transformControls.attach(baseFrameAnchor);
  scene.add(transformControls);

  // 5. 快捷键切换模式 (可选)
  window.addEventListener('keydown', (event) => {
    switch (event.key) {
      case 'g': transformControls.setMode('translate'); break; // 移动
      case 'r': transformControls.setMode('rotate'); break;    // 旋转
    }
  });
}
// *** Render the scene onto the screen ***

function render(): void {
  requestAnimationFrame(render);
  renderer.render(scene, camera);
}

function loadRobot(url = URDF_FILE_PATH, files?: Record<string, File>): void {
  if (robot) removeOldRobotFromScene();

  const filesHaveBeenUploaded = files !== undefined;
  if (filesHaveBeenUploaded) {
    loader.loadMeshCb = (
      path: string,
      manager: LoadingManager,
      onComplete: (obj: Object3D, err?: ErrorEvent) => void
    ): void => {
      const { fileName, fileExtension } = getFileNameFromPath(path);
      const fileURL = URL.createObjectURL(files[fileName]);

      switch (fileExtension) {
        case 'stl':
          loadSTL(manager, onComplete, fileURL);
          break;
        case 'dae':
          loadDAE(manager, onComplete, fileURL);
          break;
        default:
          throw new Error('Mesh format not supported');
      }
    };
  }

  loader.load(url, (result: URDFRobot): void => {
    console.log(result);
    robot = result;
  });

  // Wait until all geometry has been loaded, then add
  // the robot to the scene.
  manager.onLoad = (): void => {
    // Center the robot
    // robot.translateOnAxis(0, 0, 0);
    // robot.position.copy(new Vector3(0.0, 0.0, 0.0));
    // Traverse the robot and cast shadow
    robot.traverse((c: Object3D): void => {
      // if (c instanceof Mesh) {
      //   c.material.color.set(0xffd324);
      // }
      c.castShadow = true;
    });

    // Pass each joint's limits and initial degree to `Interface`.
    jointInfosStore.update(updateJointInfos);

    selectedUpAxisStore.update((): string => axes.Y);

    robot.scale.set(1 / scale, 1 / scale, 1 / scale);

    // Updates the global transform of the object and its descendants.
    robot.updateMatrixWorld(true);

    scene.add(robot);

    box = new THREE.Box3().setFromObject(robot);

    const savedData = get(lastPointCloudData);
    if (savedData) {
      console.log("检测到持久化点云数据，正在自动重载...");
      updateDynamicPointClouds(savedData);
    }

    loadPipe();

    // loadPointCloudA(scene, POINT_CLOUD_URLA);
    // loadPointCloudB(scene, POINT_CLOUD_URLB);

  };
}

const PIPE_STL_PATH = '../urdf/myRobot/meshes/pipe300.STL';

function loadPipe(): void {
  if (!robot) return;

  const baseLink = robot.getObjectByName('base_link');
  if (!baseLink) {
    console.warn('未找到 base_link，无法加载管道模型');
    return;
  }

  // 计算 base_link 视觉网格的几何中心
  const baseCenter = new Vector3();
  baseLink.traverse((child) => {
    if (child instanceof THREE.Mesh) {
      const geo = child.geometry;
      if (geo) {
        geo.computeBoundingBox();
        const box = geo.boundingBox!;
        baseCenter.copy(box.getCenter(new Vector3()));
      }
    }
  });

  // 清理旧管道
  if (pipeGroup) {
    baseLink.remove(pipeGroup);
    pipeGroup = null;
    pipeMesh = null;
  }

  const stlLoader = new STLLoader();
  stlLoader.load(
    PIPE_STL_PATH,
    (geometry) => {
      geometry.computeBoundingBox();
      const center = geometry.boundingBox!.getCenter(new Vector3());

      // 几何中心归零
      geometry.translate(-center.x, -center.y, -center.z);

      // 裁剪平面：在 mesh 局部空间沿 Z 截断管道（管道原始长轴为 Z）
      // 管道原始半长约 2500mm，裁剪到 ±200mm（几何空间，mm 单位）
      const CLIP_HALF = 100;
      const material = new THREE.MeshPhongMaterial({
        color: 0x888888,
        clippingPlanes: [
          new THREE.Plane(new THREE.Vector3(0, 0, -1), CLIP_HALF),
          new THREE.Plane(new THREE.Vector3(0, 0, 1), CLIP_HALF),
        ],
        clipShadows: true,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(geometry, material);

      // uniform scale: pipe300.STL 单位为 mm，URDF 模型单位为 m
      const diameter = get(pipeDiameterStore);
      const diamScale = (diameter / 100) / 1000;
      mesh.scale.set(diamScale, diamScale, diamScale / 3);

      // 旋转层：用 Group 分层旋转，避免欧拉顺序问题
      // mesh 长轴原为 Z → 绕 Y 转 -90° → 长轴变为 X
      mesh.rotation.y = -Math.PI / 2;

      const tiltGroup = new THREE.Group();
      tiltGroup.add(mesh);
      // mesh 现在沿 X，绕 Z 旋转使其向上倾斜
      tiltGroup.rotation.z = -Math.PI / 18;

      const rotGroup = new THREE.Group();
      rotGroup.add(tiltGroup);
      // 可选：微调绕 Y 的角度匹配夹持器
      // rotGroup.rotation.y = ...;

      // 放置到 base_link 视觉网格的几何中心
      rotGroup.position.copy(baseCenter);

      pipeMesh = mesh;
      pipeGroup = rotGroup;
      baseLink.add(rotGroup);
      console.log(`管道模型已加载，直径=${diameter}mm，缩放=${diamScale.toFixed(6)}`);
    },
    undefined,
    (err) => console.error('管道模型加载失败:', err)
  );
}

function removeOldRobotFromScene(): void {
  const name = scene.getObjectByName(robot.name);
  scene.remove(name);
  box = null;
  selectedUpAxisStore.update((): string => '');
  pipeMesh = null;
  pipeGroup = null;
}

// function rotateJoints(jointInfos: JointInfo[]): void {
//   if (!robot) return;
//
//   const { joints } = robot;
//   const jointNames = Object.keys(joints);
//   jointNames.forEach((jointName: string, idx: number): void => {
//     const { degree } = jointInfos[idx];
//     joints[jointName].setJointValue(MathUtils.degToRad(degree));
//   });
// }

function rotateJoints(jointInfos: JointInfo[]): void {
if (!robot || !Array.isArray(jointInfos)) return;

  const { joints } = robot;

  jointInfos.forEach((info) => {
    const jointName = info.name;
    const jointObject = joints[jointName];

    if (jointObject) {
      // 转换角度为弧度
      const rad = MathUtils.degToRad(info.degree);

      jointObject.setJointValue(rad);
    } else {
      console.warn(`模型中未找到关节: ${jointName}`);
    }
  });
}


function updateJointInfos(): JointInfo[] {
  return Object.keys(robot.joints)
    .filter((jointName) => jointName !== 'base_jointA' && jointName !== 'base_jointB')
    .map((jointName: string) => {
    const limit = robot.joints[jointName].limit || { lower: 0, upper: 0 };
    const { lower, upper } = limit;
    const lowerDegree = Number(MathUtils.radToDeg(Number(lower)).toFixed());
    const upperDegree = Number(MathUtils.radToDeg(Number(upper)).toFixed());
    const jointHasLimit = lowerDegree !== 0 || upperDegree !== 0;

    return {
      name: jointName,
      lower: jointHasLimit ? lowerDegree : -Infinity,
      upper: jointHasLimit ? upperDegree : Infinity,
      degree: 0,
    };
  });
}


function rotateRobotOnUpAxisChange(selectedUpAxis: string): void {
  if (!robot || selectedUpAxis === '') return;

  switch (selectedUpAxis) {
    case axes.X:
      setRobotRotation(robot, 0, 0, Math.PI / 2);
      break;
    case axes.NEGATIVE_X:
      setRobotRotation(robot, 0, 0, -Math.PI / 2);
      break;
    case axes.Y:
      setRobotRotation(robot, 0, 0, 0);
      break;
    case axes.NEGATIVE_Y:
      setRobotRotation(robot, Math.PI, 0, 0);
      break;
    case axes.Z:
      setRobotRotation(robot, -Math.PI / 2, 0, 0);
      break;
    case axes.NEGATIVE_Z:
      setRobotRotation(robot, Math.PI / 2, 0, 0);
      break;
    default:
      throw new Error('Should not reach here');
  }

  // If we set camera position and orbit controls' target
  // before the robot is initially rotated, robot will appear
  // off center and orbit controls will not behave correctly.
  if (!box) {
    // Create a bounding box of robot.
    box = new Box3().setFromObject(robot);

    const boxSize = box.getSize(new Vector3()).length();
    const boxCenter = box.getCenter(new Vector3());

    // robot.position.x -= box.min.y;

    scaleInView(boxSize * 0.5, boxSize, boxCenter, camera);

    controls.target.y = boxCenter.y;
    controls.update();
  }
}

function onResize(): void {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(window.devicePixelRatio);
}

export default createScene;
export { rotateJoints, loadRobot, rotateRobotOnUpAxisChange };
