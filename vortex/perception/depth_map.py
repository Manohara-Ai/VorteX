import cv2
import torch
import os
import numpy as np
from depth_anything_v2.dpt import DepthAnythingV2
from ament_index_python.packages import get_package_share_directory

_device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
_model = None

def _load_model():
    global _model
    if _model is None:
        model_config = {
            'encoder': 'vits',
            'features': 64,
            'out_channels': [48, 96, 192, 384]
        }
        _model = DepthAnythingV2(**model_config)

        # This gets the correct path at runtime
        share_dir = get_package_share_directory('vortex')
        checkpoint_path = os.path.join(share_dir, 'checkpoints', 'depth_anything_v2_vits.pth')
        
        _model.load_state_dict(torch.load(checkpoint_path, map_location='cpu'))
        _model = _model.to(_device).eval()

def get_depth_map(image: np.ndarray) -> np.ndarray:
    if image is None:
        raise ValueError("Input image is None.")
    
    _load_model()
    
    # Ensure BGR (OpenCV) to RGB conversion
    if image.shape[2] == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    with torch.no_grad():
        depth = _model.infer_image(image)
    
    return depth  # shape: (H, W)
