import * as THREE from 'three';
import {AxesHelper, Texture} from "three";

export const scale = 1.45;


let sharedCircleTexture: THREE.Texture | null = null;

function getCircleTexture(): THREE.Texture {
    if (!sharedCircleTexture) {
        const size = 64; // 64x64 对于点云贴图足够了
        const canvas = document.createElement('canvas');
        canvas.width = size;
        canvas.height = size;
        const context = canvas.getContext('2d')!;
        context.beginPath();
        context.arc(size / 2, size / 2, size / 2 - 2, 0, Math.PI * 2);
        context.fillStyle = '#ffffff';
        context.fill();

        sharedCircleTexture = new THREE.CanvasTexture(canvas);
    }
    return sharedCircleTexture;
}

/**
 * 加载并解析点云 TXT 文件
 * 假设格式为:
 * x y z
 * x y z ...
 */
/**
 * 在指定的机器人基准坐标系下加载点云
 * @param robot 加载好的 URDFRobot 或 Group 对象
 * @param url 点云文件路径
 */
export function loadPointCloudA(robot: THREE.Object3D, url: string): void {
    const loader = new THREE.FileLoader();

    // 1. 在机器人层级结构中寻找 Blender 中设定的空物体 (pc_base_frame)
    let targetFrame: THREE.Object3D | null = null;
    // robot.traverse((child) => {
    //     // 这里的名字要和你在 Blender/URDF 中设定的名字完全一致
    //     if (child.name === 'base_frameA_link') {
    //         targetFrame = child;
    //     }
    // });
    targetFrame = robot.getObjectByName('base_frameA_link');

    if (!targetFrame) {
        console.warn('未能在模型中找到 base_frameA_link，点云将默认加载至机器人原点。');
        targetFrame = robot; // 回退方案：找不到就放原点
    }
    
    if (targetFrame) {
        targetFrame.add(new AxesHelper(0.5));
    }

    loader.load(url, (data: string | ArrayBuffer) => {
        const text = data as string;
        const lines = text.split('\n');
        const positions: number[] = [];
        const colors: number[] = [];

        let count = 0;
        lines.forEach((line) => {
            if (++count % 50 !== 0) return;

            const parts = line.trim().split(',');
            if (parts.length >= 3) {
                // 这里的除以 1000 取决于你原始 TXT 的单位 (mm -> m)
                const x = parseFloat(parts[0]) / 1000;
                const y = parseFloat(parts[1]) / 1000;
                const z = parseFloat(parts[2]) / 1000;

                if (!isNaN(x) && !isNaN(y) && !isNaN(z)) {
                    // 注意：这里保留了你原来的坐标映射 positions.push(y, -x, z);
                    // 如果发现方向不对，可能需要根据 pc_base_frame 的朝向调整为 (x, y, z)
                    positions.push(x, y, z);
                }
            }
        });

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));

        const material = new THREE.PointsMaterial({
            size: 0.001,
            vertexColors: false,
            color: 0xffffff
        });

        const points = new THREE.Points(geometry, material);
        points.name = 'myPointCloud';

        // 2. 核心修改：将点云直接添加为 targetFrame 的子对象
        // 这样点云就会自动继承 targetFrame 的位置和旋转
        targetFrame.add(points);

        // 重置点云相对于父容器的局部变换
        points.position.set(0, 0, 0);
        points.rotation.set(0, 0, 0);

        points.scale.set(scale, scale, scale);

        console.log(`点云已成功附着至 [${targetFrame.name}]，共 ${positions.length / 3} 个点`);
    });
}

/**
 * 在指定的机器人基准坐标系下加载点云
 * @param robot 加载好的 URDFRobot 或 Group 对象
 * @param url 点云文件路径
 */
export function loadPointCloudB(robot: THREE.Object3D, url: string): void {
    const loader = new THREE.FileLoader();

    // 1. 在机器人层级结构中寻找 Blender 中设定的空物体 (pc_base_frame)
    let targetFrame: THREE.Object3D | null = null;

    // robot.traverse((child) => {
    //     // 这里的名字要和你在 Blender/URDF 中设定的名字完全一致
    //     // console.log(child.name);
    //     if (child.name === 'base_frameB_link') {
    //         targetFrame = child;
    //     }
    // });
    targetFrame = robot.getObjectByName('base_frameB_link');

    if (!targetFrame) {
        console.warn('未能在模型中找到 base_frameB_link，点云将默认加载至机器人原点。');
        targetFrame = robot; // 回退方案：找不到就放原点
    }

    if (targetFrame) {
        targetFrame.add(new AxesHelper(0.5));
    }

    loader.load(url, (data: string | ArrayBuffer) => {
        const text = data as string;
        const lines = text.split('\n');
        const positions: number[] = [];
        const colors: number[] = [];

        let count = 0;
        lines.forEach((line) => {
            if (++count % 50 !== 0) return;

            const parts = line.trim().split(',');
            if (parts.length >= 3) {
                // 这里的除以 1000 取决于你原始 TXT 的单位 (mm -> m)
                const x = parseFloat(parts[0]) / 1000;
                const y = parseFloat(parts[1]) / 1000;
                const z = parseFloat(parts[2]) / 1000;

                if (!isNaN(x) && !isNaN(y) && !isNaN(z)) {
                    // 注意：这里保留了你原来的坐标映射 positions.push(y, -x, z);
                    // 如果发现方向不对，可能需要根据 pc_base_frame 的朝向调整为 (x, y, z)
                    positions.push(x, y, z);
                }
            }
        });

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));

        const material = new THREE.PointsMaterial({
            size: 0.001,
            vertexColors: false,
            color: 0xffffff
        });

        const points = new THREE.Points(geometry, material);
        points.name = 'myPointCloud';

        // 2. 核心修改：将点云直接添加为 targetFrame 的子对象
        // 这样点云就会自动继承 targetFrame 的位置和旋转
        targetFrame.add(points);

        // 重置点云相对于父容器的局部变换
        points.position.set(0, 0, 0);
        points.rotation.set(0, 0, 0);
        points.scale.set(scale, scale, scale);

        console.log(`点云已成功附着至 [${targetFrame.name}]，共 ${positions.length / 3} 个点`);
    });
}
