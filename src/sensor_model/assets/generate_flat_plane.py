#%%
import open3d as o3d
import numpy as np

def create_grid_mesh(width=200, height=200, step=1):
    # 1. Create grid of vertices
    x = np.arange(0, width + step, step)
    y = np.arange(0, height + step, step)
    xv, yv = np.meshgrid(x, y)
    
    # Vertices (x, y, z=0)
    vertices = np.stack([xv.ravel(), yv.ravel(), np.zeros_like(xv).ravel()], axis=1)
    
    # 2. Define triangles for each 1x1 cell
    # A cell at (i, j) uses vertices at:
    # v0: (i, j), v1: (i+1, j), v2: (i, j+1), v3: (i+1, j+1)
    n_rows = len(y)
    n_cols = len(x)
    triangles = []
    
    for j in range(n_rows - 1):
        for i in range(n_cols - 1):
            # Calculate indices of the 4 corners
            v0 = j * n_cols + i
            v1 = v0 + 1
            v2 = (j + 1) * n_cols + i
            v3 = v2 + 1
            
            # Two triangles per cell
            triangles.append([v0, v1, v2])
            triangles.append([v1, v3, v2])

    # 3. Build the Open3D mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles))
    mesh.compute_vertex_normals()
    
    return mesh

# Example usage
terrain = create_grid_mesh(200, 200, 1)

print(f"Vertices: {len(terrain.vertices)}")
print(f"Triangles: {len(terrain.triangles)}")
print(f"Bounds: {terrain.get_min_bound()} -> {terrain.get_max_bound()}")

o3d.io.write_triangle_mesh("flatPlane_200x200m_80kTriangles.obj", terrain)
print("Saved flatPlane_200x200m_80kTriangles.obj")

o3d.visualization.draw_geometries([terrain])
