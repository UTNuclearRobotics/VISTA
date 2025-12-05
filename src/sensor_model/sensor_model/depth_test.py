#%%
import open3d as o3d
import matplotlib.pyplot as plt

# Same box setup as ray_casting_core.py create_box_scene()
position = (0, 0, 0)
width, height, depth = (1.0, 1.0, 0.5)
legacy_box = o3d.geometry.TriangleMesh.create_box(width, height, depth).translate(position)
cube = o3d.t.geometry.TriangleMesh.from_legacy(legacy_box)

# Create scene and add the cube mesh
scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(cube)

# Same ray setup as ray_casting_core.py set_rays()
rays = scene.create_rays_pinhole(
    fov_deg=90,
    center=[0, 0, 0],
    eye=[2.0, 0.0, 1.0],
    up=[0, 0, 1],
    width_px=640,
    height_px=480
)

# Compute the ray intersections
ans = scene.cast_rays(rays)

# Visualize the hit distance (depth)
plt.imshow(ans['t_hit'].numpy())
plt.colorbar(label='Depth')
plt.title('Depth Image')
plt.show()