#%%
import numpy as np

def compute_intrinsics(width_px: int, fov_x_deg: float, fov_y_deg: float) -> dict:
    """
    Compute camera intrinsics from FOV specifications.
    
    Args:
        width_px: Image width (fixed by beam count)
        fov_x_deg: Horizontal FOV in degrees
        fov_y_deg: Vertical FOV in degrees
    
    Returns:
        Dictionary with fx, fy, cx, cy, width_px, height_px, K
    """
    # Focal length from horizontal FOV
    fx = (width_px / 2.0) / np.tan(np.deg2rad(fov_x_deg / 2.0))
    fy = fx  # Square pixels
    
    # Height from vertical FOV
    height_exact = 2.0 * fy * np.tan(np.deg2rad(fov_y_deg / 2.0))
    height_px = int(np.round(height_exact))
    
    # Principal point at center
    cx = width_px / 2.0
    cy = height_px / 2.0
    
  
    
    return {
        'width_px': width_px,
        'height_px': height_px,
        'fx': fx,
        'fy': fy,
        'cx': cx,
        'cy': cy,
        'fov_x_deg': fov_x_deg,
        'fov_y_deg': fov_y_deg,
    }


# Precomputed configs for quick access
FLS_CONFIG = compute_intrinsics(width_px=512, fov_x_deg=150.0, fov_y_deg=25.0)
