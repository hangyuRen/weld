import cv2
import numpy as np
import os
import torch
import torch.nn.functional as F
from torchvision.transforms import Compose
from flask import Flask, request, jsonify

# local module imports
from depth_anything.dpt import DepthAnything
from depth_anything.util.transform import Resize, NormalizeImage, PrepareForNet

# camera parameters (fixed config)
CAMERA_PARAMS = {
    "focal_length_mm": 4,    # focal length 4mm
    "img_width": 2880,       # image width 2880 px
    "img_height": 1620,      # image height 1620 px
    "u0": 2880 / 2,          # principal point x (image center)
    "v0": 1620 / 2           # principal point y (image center)
}
# vertical height diff between camera and pipe (cm), core constant
VERTICAL_DIFF_CM = 19

FIXED_ENCODER = 'vitb'           # fixed encoder: vitb
FIXED_REF_U = 308                # fixed reference point x coordinate
FIXED_REF_V = 357                # fixed reference point y coordinate
FIXED_REF_HORIZ_DIST = 14.6      # fixed reference horizontal distance (cm)


def calculate_horizontal_distance(depth_gray, ref_info, target_coords, camera_params, vertical_diff):
    """
    Given a reference point with a known horizontal distance, compute the
    horizontal distance of a target pixel from the grayscale depth map.
    """
    u_ref, v_ref, D_ref_horiz = ref_info
    u_tar, v_tar = target_coords
    fx = camera_params["focal_length_mm"]
    u0 = camera_params["u0"]

    # check target coordinates are within the image bounds
    h, w = depth_gray.shape
    if not (0 <= u_tar < w and 0 <= v_tar < h):
        raise ValueError(f"target coords ({u_tar}, {v_tar}) out of image bounds (w:{w}, h:{h})")

    # read relative depth values of reference and target points
    d_ref = depth_gray[v_ref, u_ref]
    d_tar = depth_gray[v_tar, u_tar]

    if d_ref == 0 or d_tar == 0:
        raise ValueError("reference or target point is background (farthest), cannot compute distance")

    # step1: recover reference 3D straight-line distance from its horizontal distance (Pythagoras)
    D_ref_3d = np.sqrt(D_ref_horiz ** 2 + vertical_diff ** 2)

    # step2: compute target 3D straight-line distance (base ratio + lateral offset correction)
    D_basic = (D_ref_3d * d_ref) / d_tar
    delta_u_ref = (u_ref - u0) / fx
    delta_u_tar = (u_tar - u0) / fx
    D_tar_3d = D_basic * np.sqrt(1 + delta_u_ref ** 2) / np.sqrt(1 + delta_u_tar ** 2)

    # step3: recover target horizontal distance from its 3D straight-line distance (Pythagoras, core output)
    if D_tar_3d > vertical_diff:
        D_tar_horiz = np.sqrt(D_tar_3d ** 2 - vertical_diff ** 2)
    else:
        D_tar_horiz = 0.0  # guard against negative under root, should not happen in theory

    return D_tar_horiz


# ---------------------------------------------------------------------------
# Model / transform are loaded once at startup so that every request reuses
# them instead of reloading from disk.
# ---------------------------------------------------------------------------
model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]}
}

DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
REF_INFO = (FIXED_REF_U, FIXED_REF_V, FIXED_REF_HORIZ_DIST)
CAMERA_PARAMS_USED = CAMERA_PARAMS
VERTICAL_DIFF_USED = VERTICAL_DIFF_CM

_depth_anything = None
_transform = None


def load_model():
    """Load DepthAnything model + preprocessing transform once at startup."""
    global _depth_anything, _transform

    _depth_anything = DepthAnything(model_configs[FIXED_ENCODER])
    weight_path = f'checkpoints/depth_anything_{FIXED_ENCODER}14.pth'
    if not os.path.exists(weight_path):
        raise FileNotFoundError(f"weight missing! ensure {weight_path} exists")
    _depth_anything.load_state_dict(
        torch.load(weight_path, map_location='cpu', weights_only=True))
    _depth_anything = _depth_anything.to(DEVICE).eval()

    _transform = Compose([
        Resize(
            width=518,
            height=518,
            resize_target=False,
            keep_aspect_ratio=True,
            ensure_multiple_of=14,
            resize_method='lower_bound',
            image_interpolation_method=cv2.INTER_CUBIC,
        ),
        NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        PrepareForNet(),
    ])

    total_params = sum(p.numel() for p in _depth_anything.parameters())
    print(f'model loaded, params: {total_params / 1e6:.2f}M')
    print(f'device: {DEVICE}')
    print(f'encoder: {FIXED_ENCODER}')
    print(f'ref config: ({FIXED_REF_U}, {FIXED_REF_V}), horiz dist {FIXED_REF_HORIZ_DIST}cm')


def compute_depth_gray(raw_image):
    """
    Run DepthAnything inference on a BGR image and return the depth map
    interpolated back to the original resolution as a uint8 grayscale array.
    """
    image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB) / 255.0
    h, w = image.shape[:2]
    image = _transform({'image': image})['image']
    image = torch.from_numpy(image).unsqueeze(0).to(DEVICE)

    with torch.no_grad():
        depth = _depth_anything(image)

    # interpolate depth back to original size (keep pixel-to-pixel alignment)
    depth = F.interpolate(depth[None], (h, w), mode='bilinear', align_corners=False)[0, 0]
    depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
    return depth.cpu().numpy().astype(np.uint8)


app = Flask(__name__)


@app.route('/detect-distance', methods=['POST'])
def detect_distance():
    """Receive an image + pixel coordinates and return the horizontal distance (cm)."""
    try:
        file = request.files.get('file')
        if file is None:
            return jsonify({"distance": -1, "error": "missing file"}), 400

        pixel_x_raw = request.form.get('pixelX')
        pixel_y_raw = request.form.get('pixelY')
        if pixel_x_raw is None or pixel_y_raw is None:
            return jsonify({"distance": -1, "error": "missing pixelX/pixelY"}), 400

        pixel_x = int(float(pixel_x_raw))
        pixel_y = int(float(pixel_y_raw))

        # decode uploaded image bytes into an OpenCV BGR array
        img_bytes = np.frombuffer(file.read(), np.uint8)
        raw_image = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
        if raw_image is None:
            return jsonify({"distance": -1, "error": "invalid image"}), 400

        depth_gray = compute_depth_gray(raw_image)
        distance = calculate_horizontal_distance(
            depth_gray, REF_INFO, (pixel_x, pixel_y),
            CAMERA_PARAMS_USED, VERTICAL_DIFF_USED)

        print(f"pixel: ({pixel_x}, {pixel_y}) -> distance: {distance:.2f} cm")
        return jsonify({"distance": round(float(distance), 2)})

    except Exception as e:
        print(f"detect-distance error: {e}")
        return jsonify({"distance": -1, "error": str(e)}), 500


if __name__ == '__main__':
    load_model()
    app.run(host='0.0.0.0', port=8001, threaded=True)
