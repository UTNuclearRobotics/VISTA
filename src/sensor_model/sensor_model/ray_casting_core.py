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
        self.extrinsic = None
        
        # key is geometry ID, value is centroid coordinates array
        self.box_geometryID_hashmap = {}
        
        # hardcoding this in at the request of ARL:UT
        self.max_depth = 100 

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

    def load_mesh(self, file_path):
        """Load a mesh from file and add it to the ray casting scene.

        Args:
            file_path: Absolute path to mesh file (.obj, .stl, .ply, etc.)
        """
        mesh = o3d.io.read_triangle_mesh(file_path)
        t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        self.scene.add_triangles(t_mesh)

    def create_box_scene(self, position=(0, 0, 0), size=(1.0, 1.0, 0.5)):
        width, height, depth = size
        legacy_box = o3d.geometry.TriangleMesh.create_box(width, height, depth).translate(position)
        box = o3d.t.geometry.TriangleMesh.from_legacy(legacy_box)
        
        geometry_ID = self.scene.add_triangles(box)
        
        centroid_x = position[0] + 0.5 * size[0]
        centroid_y = position[1] +  0.5 * size[1]
        centroid_z = position[2] +  0.5 * size[2]
        
        self.box_geometryID_hashmap[geometry_ID] = [centroid_x,centroid_y,centroid_z]
        
        
        
    
    def create_monkey_scene(self, position=(0, 0, 0), scale=1.0):
        monkey_data = o3d.data.MonkeyModel()
        mesh = o3d.io.read_triangle_mesh(monkey_data.path)
        mesh.scale(scale, center=mesh.get_center())
        mesh.translate(position)
        t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        self.scene.add_triangles(t_mesh)
    
    def set_rays(self, extrinsic):
        self.extrinsic = extrinsic # use for ping()
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
        hits = self.scene.cast_rays(self.rays)
        
        hit_ids = hits['geometry_ids'].numpy().flatten()
        dists = hits['t_hit']  # shape (H, W)
        
        #in the future create a boolean mask based on minimum and max distances
        # introduce gaussian noise if needed
        
        valid_dists = (dists.isfinite()) & (dists <= self.max_depth)
        dists[valid_dists.logical_not()] = 0.0
        dists = dists.numpy().flatten()  # numpy (H*W,)
        rays = self.rays.numpy().reshape(-1, 6)  # numpy copy (H*W, 6)

        # convert direction vectors to displacement vectors
        rays[:, 3] *= dists
        rays[:, 4] *= dists
        rays[:, 5] *= dists

        points = rays[:, 0:3] + rays[:, 3:]  # defined in the simulation frame

        R = self.extrinsic[:3, :3]  # world-to-camera rotation
        t = self.extrinsic[:3, 3]   # world-to-camera translation

        # points is (H*W, 3), transpose to (3, H*W) for R @, then back
        local_points = (R @ points.T).T

        # apply translation to each column, element wise
        local_points[:, 0] += t[0]
        local_points[:, 1] += t[1]
        local_points[:, 2] += t[2]

        # retrieve z component of points as depth
        depth = local_points[:, 2].reshape(self.height_px, self.width_px)
        
        # --- process hits: extract geometry + primitive (triangle) ids ---
        prim_ids = hits['primitive_ids'].numpy().flatten()   # triangle index per ray

        # valid-hit mask (in range, not a miss) — apply to BOTH arrays in lockstep
        valid = (dists > 0.0) & (hit_ids != 0xFFFFFFFF)
        geom_valid = hit_ids[valid]
        prim_valid = prim_ids[valid]

        # CIR: per-frame UNIQUE (geometry_id, primitive_id) pairs — ALL geoms incl. seabed.
        face_pairs = np.unique(np.stack([geom_valid, prim_valid], axis=1), axis=0)  # (M, 2)

        # detection (existing behavior): box geometry IDs only
        box_mask = np.isin(geom_valid, list(self.box_geometryID_hashmap.keys()))
        box_ids = np.unique(geom_valid[box_mask])

        return depth, box_ids, face_pairs










