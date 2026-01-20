import numpy as np
import open3d as o3d


class RayCastingCore:
    def __init__(self, width_px, height_px, fx, fy, cx=None, cy=None):
        self.scene = o3d.t.geometry.RaycastingScene()
        
        #intrinsics here, use these attributes to populate CameraInfo message directly
        self.width_px = width_px
        self.height_px = height_px
        self.fx = fx
        self.fy = fy
        self.cx = cx if cx is not None else width_px / 2.0
        self.cy = cy if cy is not None else height_px / 2.0
        
        self.rays = None

    @classmethod
    def from_config(cls, config: dict):
        """initialize object from a config dict (e.g FLS_CONFIG)"""
        return cls(
            width_px=config['width_px'],
            height_px=config['height_px'],
            fx=config['fx'],
            fy=config['fy'],
            cx=config['cx'],
            cy=config['cy'],
        )

    @property
    def K(self):
        """3x3 intrinsic matrix"""
        return np.array([
            [self.fx, 0,       self.cx],
            [0,       self.fy, self.cy],
            [0,       0,       1      ]
        ], dtype=np.float64)

    def create_box_scene(self, position=(0, 0, 0), size=(1.0, 1.0, 0.5)):
        width, height, depth = size
        legacy_box = o3d.geometry.TriangleMesh.create_box(width, height, depth).translate(position)
        box = o3d.t.geometry.TriangleMesh.from_legacy(legacy_box)
        self.scene.add_triangles(box)
    
    def set_rays(self, extrinsic):
        """Set rays with given extrinsic matrix (world pose to camera frame).

        Args:
            extrinsic: 4x4 numpy array representing world-to-camera transform.

        Camera convention: looks along +Z in camera frame.
        """
        self.rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            intrinsic_matrix=self.K,
            extrinsic_matrix=extrinsic,
            width_px=self.width_px,
            height_px=self.height_px,
        )
    
    def ping(self):
        """Cast rays through the scene and return depth image.

        Returns:
            np.ndarray: Depth image of shape (H, W) with invalid depths set to 0.0
        """
        ans = self.scene.cast_rays(self.rays)
        depth = ans['t_hit'].numpy()   # shape (H, W)
        depth[~np.isfinite(depth)] = 0.0
        return depth










