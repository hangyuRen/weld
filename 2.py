from fastapi import FastAPI, UploadFile, File, Form
from fastapi.responses import JSONResponse, FileResponse
from ultralytics import YOLO
import cv2
import numpy as np
import uuid
import os
import asyncio
import logging
import time
import torch
import torch.nn.functional as F
from torchvision.transforms import Compose

# 深度模型本地模块导入（确保dpt.py和util在项目根目录）
from depth_anything.dpt import DepthAnything
from depth_anything.util.transform import Resize, NormalizeImage, PrepareForNet

# ====================== 日志配置 ======================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)
logger = logging.getLogger("detect-distance")

# ====================== 全局配置（可根据需求修改）======================
app = FastAPI(title="目标检测+深度距离计算API", version="1.0")

# YOLO模型路径
YOLO_MODEL_PATH = "model/best.pt"
# YOLO高召回配置：优先尽可能检出焊缝，代价是速度下降且误检增加。
# 如果误检过多，优先把YOLO_CONFIDENCE调高到0.03或0.05。
YOLO_CONFIDENCE = 0.01
YOLO_IMAGE_SIZE = 1280
YOLO_IOU = 0.70
YOLO_MAX_DETECTIONS = 100
YOLO_AUGMENT = True
# 深度模型配置
DEPTH_ENCODER = 'vitb'
DEPTH_WEIGHT_PATH = f'checkpoints/depth_anything_{DEPTH_ENCODER}14.pth'
# 参照物配置（核心！根据你的实际场景修改像素坐标和实际距离）
FIXED_REF_U = 308
FIXED_REF_V = 357
FIXED_REF_HORIZ_DIST = 13.6  # 参照物实际水平距离 (cm)
VERTICAL_DIFF_CM = 15       # 相机与管道垂直高度差 (cm)
# 检测红点偏移量（检测框上边向下偏移像素）
OFFSET_Y = 15
# 路径配置
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, "outputs")
os.makedirs(OUTPUT_DIR, exist_ok=True)
# 设备配置（自动使用CUDA/CPU）
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

# YOLO和深度模型是全局共享对象。使用异步锁确保同一时间只有一个请求执行
# 模型推理；等待锁和工作线程执行期间不会阻塞Uvicorn事件循环。
INFERENCE_LOCK = asyncio.Lock()

# ====================== 全局模型加载（容器/服务启动时仅加载一次，提升性能）======================
# 1. 加载YOLO目标检测模型
try:
    logger.info("开始加载YOLO模型: %s", YOLO_MODEL_PATH)
    load_start = time.perf_counter()
    yolo_model = YOLO(YOLO_MODEL_PATH)
    logger.info(
        "YOLO模型加载完成: %s | 设备: %s | 耗时: %.3fs",
        YOLO_MODEL_PATH, DEVICE, time.perf_counter() - load_start
    )
except Exception as e:
    logger.exception("YOLO模型加载失败")
    raise FileNotFoundError(f"YOLO模型加载失败: {e}")

# 2. 加载深度估计模型
try:
    logger.info("开始加载深度模型: %s", DEPTH_WEIGHT_PATH)
    load_start = time.perf_counter()
    model_configs = {
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    }
    depth_model = DepthAnything(model_configs[DEPTH_ENCODER])
    depth_model.load_state_dict(torch.load(DEPTH_WEIGHT_PATH, map_location='cpu', weights_only=True))
    depth_model = depth_model.to(DEVICE).eval()
    # 深度模型图像预处理管线
    depth_transform = Compose([
        Resize(width=518, height=518, resize_target=False, keep_aspect_ratio=True,
               ensure_multiple_of=14, resize_method='lower_bound',
               image_interpolation_method=cv2.INTER_CUBIC),
        NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        PrepareForNet(),
    ])
    logger.info(
        "深度模型加载完成: %s | 设备: %s | 耗时: %.3fs",
        DEPTH_WEIGHT_PATH, DEVICE, time.perf_counter() - load_start
    )
except Exception as e:
    logger.exception("深度模型加载失败")
    raise FileNotFoundError(f"深度模型加载失败: {e}")

# ====================== 核心工具函数（和原有脚本逻辑一致，适配接口）======================
def get_best_bbox(yolo_results):
    """从YOLO结果中获取置信度最高的检测框，计算红点坐标"""
    detections = []
    best_det = None
    max_conf = 0.0

    for box in yolo_results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        cls = int(box.cls[0])
        label = yolo_model.names[cls]

        det = {
            "label": label,
            "confidence": conf,
            "bbox": [x1, y1, x2, y2],
            "point": {
                "x": (x1 + x2) / 2,  # 检测框上边中点x
                "y": y1 + OFFSET_Y   # 检测框上边中点y + 偏移
            }
        }
        detections.append(det)

        # 筛选置信度最高的检测框
        if conf > max_conf:
            max_conf = conf
            best_det = det

    return best_det, detections

def calculate_horizontal_distance(depth_gray, ref_u, ref_v, ref_horiz_dist, target_x, target_y,
                                  img_w, img_h, vertical_diff):
    """计算目标相对于参照物的水平距离（适配接口，动态传参）"""
    # 相机参数（动态根据图片尺寸生成）
    fx = img_w  # 像素焦距粗略近似，可根据实际相机参数调整
    u0 = img_w / 2
    v0 = img_h / 2

    # 坐标越界保护（防止像素坐标超出图片范围）
    u_tar = max(0, min(int(target_x), img_w - 1))
    v_tar = max(0, min(int(target_y), img_h - 1))
    u_ref = max(0, min(int(ref_u), img_w - 1))
    v_ref = max(0, min(int(ref_v), img_h - 1))

    # 读取深度值
    d_ref = float(depth_gray[v_ref, u_ref])
    d_tar = float(depth_gray[v_tar, u_tar])

    # 深度值合法性校验
    if d_ref < 1e-5:
        return 0.0, "参照物位置深度为0，无法计算"
    if d_tar < 1e-5:
        return 0.0, "目标位置深度为0，计算结果无效"

    # 步骤1：参照物3D直线距离
    D_ref_3d = np.sqrt(ref_horiz_dist ** 2 + vertical_diff ** 2)
    # 步骤2：目标3D直线距离（基础比例）
    D_basic = (D_ref_3d * d_ref) / d_tar
    # 步骤3：角度修正（使用像素焦距）
    delta_u_ref = (u_ref - u0) / fx
    delta_u_tar = (u_tar - u0) / fx
    factor_ref = np.sqrt(1 + delta_u_ref ** 2)
    factor_tar = np.sqrt(1 + delta_u_tar ** 2)
    D_tar_3d = D_basic * factor_ref / factor_tar
    # 步骤4：反推水平距离
    if D_tar_3d > vertical_diff:
        D_tar_horiz = np.sqrt(D_tar_3d ** 2 - vertical_diff ** 2)
        return round(D_tar_horiz, 2), "计算成功"
    else:
        return 0.0, "目标3D距离小于垂直高度差，水平距离为0"

def draw_result(img, det_result, horizontal_dist, depth_gray):
    """绘制检测框、红点、距离文本，生成结果图和深度图"""
    # 绘制YOLO检测框和红点
    x1, y1, x2, y2 = map(int, det_result["bbox"])
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    px, py = int(det_result["point"]["x"]), int(det_result["point"]["y"])
    cv2.circle(img, (px, py), 5, (0, 0, 255), -1)

    # 绘制文本（检测标签+置信度、水平距离）
    det_text = f'{det_result["label"]} {det_result["confidence"]:.2f}'
    dist_text = f"Horiz Dist: {horizontal_dist:.2f} cm"
    cv2.putText(img, det_text, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(img, dist_text, (px + 10, py), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # 生成彩色深度图
    depth_colored = cv2.applyColorMap(depth_gray, cv2.COLORMAP_INFERNO)

    return img, depth_colored


def run_yolo_inference(raw_img):
    """在线程池中执行同步YOLO推理。"""
    return yolo_model.predict(
        source=raw_img,
        conf=YOLO_CONFIDENCE,
        imgsz=YOLO_IMAGE_SIZE,
        iou=YOLO_IOU,
        max_det=YOLO_MAX_DETECTIONS,
        augment=YOLO_AUGMENT,
        verbose=False,
    )[0]


def draw_and_save_results(draw_img, best_det, horizontal_dist, depth_gray,
                          result_img_path, depth_img_path):
    """在线程池中绘制并保存结果图片。"""
    result_img, depth_img = draw_result(
        draw_img, best_det, horizontal_dist, depth_gray
    )
    result_saved = cv2.imwrite(result_img_path, result_img)
    depth_saved = cv2.imwrite(depth_img_path, depth_img)
    if not result_saved or not depth_saved:
        raise IOError(
            f"结果图片保存失败: result_saved={result_saved}, "
            f"depth_saved={depth_saved}"
        )

# ====================== 核心接口：上传图片→一站式处理→返回距离+结果图 =======================
@app.post("/detect-distance", summary="上传图片，检测目标并计算水平距离")
async def detect_and_calculate(file: UploadFile = File(..., description="上传需要处理的图片（jpg/png）")):
    request_id = uuid.uuid4().hex[:8]
    request_start = time.perf_counter()
    logger.info("[%s] 收到/detect-distance请求 | 文件名: %s | 类型: %s",
                request_id, file.filename, file.content_type)
    try:
        # 1. 读取上传的图片，转换为OpenCV格式
        step_start = time.perf_counter()
        logger.info("[%s] 开始读取上传文件", request_id)
        image_bytes = await file.read()
        logger.info("[%s] 文件读取完成 | 大小: %d bytes | 耗时: %.3fs",
                    request_id, len(image_bytes), time.perf_counter() - step_start)

        step_start = time.perf_counter()
        logger.info("[%s] 开始解码图片", request_id)
        np_img = np.frombuffer(image_bytes, np.uint8)
        raw_img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        if raw_img is None:
            logger.warning("[%s] 图片解码失败 | 总耗时: %.3fs",
                           request_id, time.perf_counter() - request_start)
            return JSONResponse(status_code=400, content={"error": "图片格式错误，无法解析"})
        img_h, img_w = raw_img.shape[:2]
        logger.info("[%s] 图片解码完成 | 尺寸: %dx%d | 耗时: %.3fs",
                    request_id, img_w, img_h, time.perf_counter() - step_start)
        # 备份原图用于绘制结果
        draw_img = raw_img.copy()

        # 2. YOLO目标检测（获取最佳检测框和红点坐标）
        step_start = time.perf_counter()
        logger.info("[%s] 等待获取模型推理锁", request_id)
        async with INFERENCE_LOCK:
            logger.info("[%s] 已获取模型推理锁 | 等待: %.3fs",
                        request_id, time.perf_counter() - step_start)

            logger.info("[%s] 开始YOLO目标检测（工作线程）", request_id)
            step_start = time.perf_counter()
            yolo_results = await asyncio.to_thread(run_yolo_inference, raw_img)
            logger.info("[%s] YOLO推理完成 | 耗时: %.3fs",
                        request_id, time.perf_counter() - step_start)
            best_det, all_detections = get_best_bbox(yolo_results)
            logger.info("[%s] 检测结果解析完成 | 目标数: %d",
                        request_id, len(all_detections))
            if best_det is None:
                logger.warning("[%s] 未检测到目标，返回404 | 总耗时: %.3fs",
                               request_id, time.perf_counter() - request_start)
                return JSONResponse(
                    status_code=404,
                    content={"error": "未检测到任何目标，无法计算距离"}
                )
            target_x = best_det["point"]["x"]
            target_y = best_det["point"]["y"]
            logger.info(
                "[%s] 最佳目标 | 标签: %s | 置信度: %.4f | 目标点: (%.1f, %.1f)",
                request_id, best_det["label"], best_det["confidence"],
                target_x, target_y
            )

            # 3. 深度估计（生成原图尺寸的深度灰度图）
            step_start = time.perf_counter()
            logger.info("[%s] 开始深度估计（工作线程）", request_id)
            depth_gray = await asyncio.to_thread(
                compute_depth_gray, raw_img, request_id
            )
            logger.info("[%s] 深度估计完成 | 耗时: %.3fs",
                        request_id, time.perf_counter() - step_start)

        logger.info("[%s] 已释放模型推理锁", request_id)

        # 4. 计算水平距离
        step_start = time.perf_counter()
        logger.info("[%s] 开始计算水平距离", request_id)
        horizontal_dist, calc_msg = calculate_horizontal_distance(
            depth_gray=depth_gray,
            ref_u=FIXED_REF_U,
            ref_v=FIXED_REF_V,
            ref_horiz_dist=FIXED_REF_HORIZ_DIST,
            target_x=target_x,
            target_y=target_y,
            img_w=img_w,
            img_h=img_h,
            vertical_diff=VERTICAL_DIFF_CM
        )
        logger.info("[%s] 距离计算完成 | 距离: %.2f cm | 状态: %s | 耗时: %.3fs",
                    request_id, horizontal_dist, calc_msg, time.perf_counter() - step_start)

        # 5. 绘制结果图，生成唯一文件名（避免重复）
        uuid_str = uuid.uuid4().hex
        # 检测结果图（带框、红点、距离）
        result_img_path = os.path.join(OUTPUT_DIR, f"{uuid_str}_result.jpg")
        # 彩色深度图
        depth_img_path = os.path.join(OUTPUT_DIR, f"{uuid_str}_depth.jpg")
        # 绘制并保存
        step_start = time.perf_counter()
        logger.info("[%s] 开始绘制并保存结果图片（工作线程）", request_id)
        await asyncio.to_thread(
            draw_and_save_results,
            draw_img,
            best_det,
            horizontal_dist,
            depth_gray,
            result_img_path,
            depth_img_path,
        )
        logger.info("[%s] 结果图片保存完成 | result: %s | depth: %s | 耗时: %.3fs",
                    request_id, result_img_path, depth_img_path,
                    time.perf_counter() - step_start)

        # 6. 构造返回结果（包含距离、检测信息、图片访问路径）
        logger.info("[%s] 请求处理完成，准备返回200 | 总耗时: %.3fs",
                    request_id, time.perf_counter() - request_start)
        return JSONResponse(content={
            "code": 200,
            "msg": calc_msg,
            "distance_m": horizontal_dist*0.01,  # 核心：水平距离（厘米）
            "best_detection": best_det,                # 置信度最高的检测结果
            "all_detections": all_detections,          # 所有检测结果
            "result_image": f"/result/{os.path.basename(result_img_path)}",  # 结果图访问路径
            "depth_image": f"/result/{os.path.basename(depth_img_path)}"     # 深度图访问路径
        })

    except Exception as e:
        # 全局异常捕获，返回错误信息
        logger.exception("[%s] 请求处理失败 | 总耗时: %.3fs",
                         request_id, time.perf_counter() - request_start)
        return JSONResponse(status_code=500, content={"error": f"处理失败: {str(e)}"})

# ====================== 原有接口：根据文件名获取结果图（复用）======================
@app.get("/result/{image_name}", summary="根据文件名获取检测/深度结果图")
def get_result(image_name: str):
    file_path = os.path.join(OUTPUT_DIR, image_name)
    if not os.path.exists(file_path):
        return JSONResponse(status_code=404, content={"error": "图片不存在"})
    # 根据后缀判断媒体类型，适配jpg/png
    media_type = "image/jpeg" if image_name.endswith((".jpg", ".jpeg")) else "image/png"
    return FileResponse(
        file_path,
        media_type=media_type,
        filename=image_name
    )

# ====================== 手动模式接口（对应 1.py：像素坐标计算距离）======================
MANUAL_FIXED_REF_U = 308
MANUAL_FIXED_REF_V = 357
MANUAL_FIXED_REF_HORIZ_DIST = 14.6   # 参照物实际水平距离 (cm)
MANUAL_VERTICAL_DIFF_CM = 19         # 相机与管道垂直高度差 (cm)
MANUAL_FOCAL_LENGTH_MM = 4.0         # 焦距 mm
MANUAL_U0 = 2880 / 2                 # 主点 x（图像中心）


def compute_depth_gray(raw_img, request_id="-"):
    """复用已加载的深度模型，计算原图尺寸的深度灰度图"""
    img_h, img_w = raw_img.shape[:2]

    step_start = time.perf_counter()
    logger.info("[%s] 深度预处理开始", request_id)
    rgb_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2RGB) / 255.0
    transformed_img = depth_transform({'image': rgb_img})['image']
    tensor_img = torch.from_numpy(transformed_img).unsqueeze(0).to(DEVICE)
    logger.info("[%s] 深度预处理完成 | tensor形状: %s | 耗时: %.3fs",
                request_id, tuple(tensor_img.shape), time.perf_counter() - step_start)

    step_start = time.perf_counter()
    logger.info("[%s] 深度模型推理开始 | 设备: %s", request_id, DEVICE)
    with torch.no_grad():
        depth = depth_model(tensor_img)
    if DEVICE == 'cuda':
        torch.cuda.synchronize()
    logger.info("[%s] 深度模型推理完成 | 耗时: %.3fs",
                request_id, time.perf_counter() - step_start)

    step_start = time.perf_counter()
    logger.info("[%s] 深度图插值和归一化开始 | 输出尺寸: %dx%d",
                request_id, img_w, img_h)
    depth = F.interpolate(depth[None], (img_h, img_w), mode='bilinear', align_corners=False)[0, 0]
    depth_min = depth.min()
    depth_max = depth.max()
    depth_range = depth_max - depth_min
    if depth_range.item() < 1e-8:
        raise ValueError("深度模型输出范围为0，无法归一化")
    depth = (depth - depth_min) / depth_range * 255.0
    depth_gray = depth.cpu().numpy().astype(np.uint8)
    logger.info("[%s] 深度图插值和归一化完成 | 范围: %.6f~%.6f | 耗时: %.3fs",
                request_id, depth_min.item(), depth_max.item(),
                time.perf_counter() - step_start)
    return depth_gray


def calculate_manual_distance(depth_gray, pixel_x, pixel_y):
    """根据参照物与目标像素，计算目标水平距离（对应 1.py 的手动算法）"""
    u_ref, v_ref, d_ref_horiz = MANUAL_FIXED_REF_U, MANUAL_FIXED_REF_V, MANUAL_FIXED_REF_HORIZ_DIST
    u_tar, v_tar = pixel_x, pixel_y
    fx = MANUAL_FOCAL_LENGTH_MM
    u0 = MANUAL_U0
    vertical_diff = MANUAL_VERTICAL_DIFF_CM

    h, w = depth_gray.shape
    if not (0 <= u_tar < w and 0 <= v_tar < h):
        raise ValueError(f"target coords ({u_tar}, {v_tar}) out of image bounds (w:{w}, h:{h})")

    d_ref = float(depth_gray[v_ref, u_ref])
    d_tar = float(depth_gray[v_tar, u_tar])
    if d_ref == 0 or d_tar == 0:
        raise ValueError("reference or target point is background, cannot compute distance")

    # 步骤1：参照物 3D 直线距离
    D_ref_3d = np.sqrt(d_ref_horiz ** 2 + vertical_diff ** 2)
    # 步骤2：目标 3D 直线距离（基础比例）
    D_basic = (D_ref_3d * d_ref) / d_tar
    # 步骤3：角度修正
    delta_u_ref = (u_ref - u0) / fx
    delta_u_tar = (u_tar - u0) / fx
    D_tar_3d = D_basic * np.sqrt(1 + delta_u_ref ** 2) / np.sqrt(1 + delta_u_tar ** 2)
    # 步骤4：反推水平距离
    if D_tar_3d > vertical_diff:
        return float(np.sqrt(D_tar_3d ** 2 - vertical_diff ** 2))
    return 0.0


@app.post("/detect-distance/manual", summary="上传图片+像素坐标，计算水平距离")
async def detect_distance_manual(
    file: UploadFile = File(..., description="上传需要处理的图片（jpg/png）"),
    pixelX: str = Form(...),
    pixelY: str = Form(...),
):
    request_id = uuid.uuid4().hex[:8]
    request_start = time.perf_counter()
    logger.info("[%s] 收到/detect-distance/manual请求 | 文件名: %s | pixelX: %s | pixelY: %s",
                request_id, file.filename, pixelX, pixelY)
    try:
        # 1. 读取上传的图片
        step_start = time.perf_counter()
        image_bytes = await file.read()
        logger.info("[%s] 手动模式文件读取完成 | 大小: %d bytes | 耗时: %.3fs",
                    request_id, len(image_bytes), time.perf_counter() - step_start)
        np_img = np.frombuffer(image_bytes, np.uint8)
        raw_img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        if raw_img is None:
            logger.warning("[%s] 手动模式图片解码失败", request_id)
            return JSONResponse(status_code=400, content={"distance": -1, "error": "图片格式错误，无法解析"})

        pixel_x = int(float(pixelX))
        pixel_y = int(float(pixelY))

        # 2. 深度估计 + 距离计算
        lock_start = time.perf_counter()
        logger.info("[%s] 手动模式等待获取模型推理锁", request_id)
        async with INFERENCE_LOCK:
            logger.info("[%s] 手动模式已获取模型推理锁 | 等待: %.3fs",
                        request_id, time.perf_counter() - lock_start)
            logger.info("[%s] 手动模式开始深度估计（工作线程）", request_id)
            depth_gray = await asyncio.to_thread(
                compute_depth_gray, raw_img, request_id
            )
        logger.info("[%s] 手动模式已释放模型推理锁", request_id)
        distance = calculate_manual_distance(depth_gray, pixel_x, pixel_y)

        logger.info("[%s] 手动模式处理完成 | pixel: (%d, %d) | 距离: %.2f cm | 总耗时: %.3fs",
                    request_id, pixel_x, pixel_y, distance,
                    time.perf_counter() - request_start)
        return {"distance": round(distance, 2)}
    except Exception as e:
        logger.exception("[%s] 手动模式处理失败 | 总耗时: %.3fs",
                         request_id, time.perf_counter() - request_start)
        return JSONResponse(status_code=500, content={"distance": -1, "error": str(e)})


# 启动服务提示（可选）
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
