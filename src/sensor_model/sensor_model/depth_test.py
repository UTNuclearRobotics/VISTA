#%%
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Test 1: Camera at origin, identity transform (looking along +Z)
# Box offset to be in front of camera
position = (0, 0, 1)  # Place box 1m away in +Z direction
width, height, depth = (1.0, 1.0, 0.5)
legacy_box = o3d.geometry.TriangleMesh.create_box(width, height, depth).translate(position)
cube = o3d.t.geometry.TriangleMesh.from_legacy(legacy_box)

scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(cube)

intrinsic = np.array([
    [68.6, 0.0, 256.0],
    [0.0, 68.6, 15.0],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

# Identity extrinsic - camera at origin looking along +Z
extrinsic = np.eye(4, dtype=np.float64)

rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
    intrinsic_matrix=intrinsic,
    extrinsic_matrix=extrinsic,
    width_px=512,
    height_px=30
)

ans = scene.cast_rays(rays)
depth = ans['t_hit'].numpy()

valid_depths = depth[depth > 0]
if len(valid_depths) > 0:
    print(f"Depth range: {valid_depths.min():.3f}m to {valid_depths.max():.3f}m")
else:
    print("No valid depths detected")

plt.imshow(depth)
plt.colorbar(label='Depth (meters)')
plt.title('Camera at origin, identity extrinsic')
plt.show()