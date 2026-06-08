#! /usr/bin/env python3
#
# MegaLoc global descriptor extractor for RTAB-Map.
# Paper: "MegaLoc: One Retrieval to Place Them All" (Berton & Masone, CVPRW 2025)
# Model: https://github.com/gmberton/MegaLoc
#
# Requirements:
#   pip install torch torchvision huggingface_hub safetensors
#
# Usage with rtabmap:
#   --Mem/GlobalDescriptorStrategy 1 --Kp/TfIdfLikelihoodUsed false \
#   --Mem/RehearsalSimilarity 1 \
#   --PyDescriptor/Dim 8448 \
#   --PyDescriptor/Path /path/to/rtabmap_megaloc.py
#

import sys
import os
import numpy as np

if not hasattr(sys, 'argv'):
    sys.argv = ['']

import torch
import torchvision.transforms.functional as TF

model = None
device = None

# DINOv2 backbone expects ImageNet-normalized inputs
_MEAN = [0.485, 0.456, 0.406]
_STD  = [0.229, 0.224, 0.225]


def init(descriptorDim):
    global model, device
    print("MegaLoc python init()")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = torch.hub.load("gmberton/MegaLoc", "get_trained_model")
    model = model.eval().to(device)
    print(f"MegaLoc initialized on {device}, descriptor dim={descriptorDim}")


def extract(image):
    # image is a HxWxC uint8 numpy array in BGR (OpenCV convention)
    global model, device

    if image.ndim == 2 or image.shape[2] == 1:
        gray = image.squeeze() if image.ndim == 3 else image
        rgb = np.stack([gray, gray, gray], axis=2)
    else:
        # BGR -> RGB
        rgb = image[:, :, ::-1].copy()

    # HxWx3 uint8  ->  3xHxW float32 in [0, 1]
    tensor = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
    tensor = TF.normalize(tensor, mean=_MEAN, std=_STD)
    tensor = tensor.unsqueeze(0).to(device)  # 1x3xHxW

    with torch.no_grad():
        descriptor = model(tensor)  # 1x8448, already L2-normalized by model

    return np.float32(descriptor.cpu().numpy())


if __name__ == '__main__':
    img = np.zeros([224, 224, 3], dtype=np.uint8)
    init(8448)
    descriptor = extract(img)
    print(f"Descriptor shape: {descriptor.shape}")
    print(f"Descriptor norm:  {np.linalg.norm(descriptor):.4f}")  # should be ~1.0
