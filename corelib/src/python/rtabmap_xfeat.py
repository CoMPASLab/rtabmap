import torch
import numpy as np

from modules.xfeat import XFeat

device = "cpu"

def init(cuda):
    # Set device
    global device
    device = "cuda" if torch.cuda.is_available() and cuda else "cpu"
    
    # Disable FutureWarning from PyTorch
    import warnings
    warnings.simplefilter("ignore", FutureWarning)

    # Initialize xfeat
    global xfeat
    xfeat = XFeat().to(device)


def detect(imageBuffer):
    # Image is provided as WH but xfeat expects BCHW
    global device
    image = np.asarray(imageBuffer)
    image_tensor = torch.from_numpy(image).reshape(1, 1, image.shape[1], image.shape[0]).float().to(device)

    # Run xfeat
    global xfeat
    output = xfeat.detectAndCompute(image_tensor)[0]
    
    # Keypoints are (N,2), we need (N,3) with last column=1 for RTAB-Map (representing confidence)
    keypoints = output["keypoints"].cpu().float().numpy()
    keypoints = np.concatenate([keypoints, np.ones((keypoints.shape[0], 1), dtype=keypoints.dtype)], axis=1)
    
    # Descriptors are (N,64), need to be unit-normalized
    descriptors = output["descriptors"].cpu().float().numpy()
    norms = np.linalg.norm(descriptors, axis=1, keepdims=True)
    norms[norms == 0] = 1  # Prevent division by zero
    descriptors /= norms

    return keypoints, descriptors


if __name__ == "__main__":
    init(True)
    k, d = detect(np.random.rand(640,480) * 255)
    print(f"Detected {k.shape[0]} keypoints")
    print(f"Keypoints shape: {k.shape}, Descriptors shape: {d.shape}")
    
    # Benchmark speed by running 100 iterations
    import time
    print("Benchmarking speed...")
    start = time.time()
    n = 100
    for i in range(n):
        k, d = detect(np.random.rand(640,480) * 255)
    end = time.time()
    print(f"Average time: {(end-start) * 1000 / n:.3f} ms")