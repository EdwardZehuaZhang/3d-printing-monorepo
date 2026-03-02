"""
Visualize single-stroke path on the Stanford Bunny surface.
Generates GraphLattice trace only (no body). Use viz.Export to write STL if needed.
"""
import pyvcad as pv
import pyvcad_rendering as viz
import numpy as np
from pathlib import Path
from scipy.spatial import distance_matrix
import trimesh


def load_mesh_vertices(mesh_path: Path) -> np.ndarray:
    """Load vertices; use simple OBJ parser for .obj to match original behavior, trimesh for .ply."""
    if mesh_path.suffix.lower() == ".obj":
        verts = []
        with open(mesh_path, "r") as f:
            for line in f:
                if line.startswith("v "):
                    _, x, y, z = line.strip().split()
                    verts.append([float(x), float(y), float(z)])
        return np.array(verts)
    # fallback: trimesh for other formats (e.g., .ply)
    mesh = trimesh.load_mesh(str(mesh_path), process=False)
    return np.asarray(mesh.vertices)


def sample_surface_points(vertices: np.ndarray, num_points: int) -> np.ndarray:
    idx = np.random.choice(len(vertices), size=min(num_points, len(vertices)), replace=False)
    return vertices[idx]


def build_knn_edges(points: np.ndarray, k: int):
    dist = distance_matrix(points, points)
    edges = []
    for i in range(len(points)):
        nbrs = np.argsort(dist[i])[: k + 1]
        for j in nbrs[1:]:
            if i < j:
                edges.append((i, j, dist[i, j]))
    return edges


def kruskal_mst(points: np.ndarray, edges):
    edges = sorted(edges, key=lambda e: e[2])
    parent = list(range(len(points)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra == rb:
            return False
        parent[ra] = rb
        return True

    mst = []
    for i, j, _w in edges:
        if union(i, j):
            mst.append((i, j))
        if len(mst) == len(points) - 1:
            break
    return mst


def mst_to_path(mst_edges, n):
    adj = [[] for _ in range(n)]
    for a, b in mst_edges:
        adj[a].append(b)
        adj[b].append(a)
    seen = [False] * n
    path = []

    def dfs(node):
        seen[node] = True
        path.append(node)
        for nxt in adj[node]:
            if not seen[nxt]:
                dfs(nxt)
                path.append(node)

    dfs(0)
    return path


def generate_curve_points(mesh_path: Path, num_points=500, k_neighbors=5):
    print(f"Loading mesh vertices from: {mesh_path}")
    verts = load_mesh_vertices(mesh_path)
    print(f"Loaded {len(verts)} vertices")
    pts = sample_surface_points(verts, num_points)
    print(f"Sampling {num_points} surface points...")
    edges = build_knn_edges(pts, k_neighbors)
    print(f"Building k-NN graph (k={k_neighbors})...")
    print(f"Generated {len(edges)} edges")
    mst = kruskal_mst(pts, edges)
    print(f"Building minimum spanning tree...")
    print(f"MST has {len(mst)} edges")
    order = mst_to_path(mst, len(pts))
    print("Converting MST to continuous path...")
    print(f"Generated path with {len(order)} points")
    return [pv.Vec3(*pts[i]) for i in order]


def main():
    script_dir = Path(__file__).parent
    ply_file = script_dir / "base-models" / "bunny_watertight.ply"
    obj_file = script_dir / "base-models" / "bunny.obj"
    mesh_file = ply_file if ply_file.exists() else obj_file
    materials = pv.default_materials
    curve_pts = generate_curve_points(mesh_file, num_points=500, k_neighbors=5)
    struts = [(curve_pts[i], curve_pts[i + 1]) for i in range(len(curve_pts) - 1)]
    print(f"Creating GraphLattice with {len(struts)} struts...")
    strut_diam = 0.0015  # 1.5mm for better visibility
    print(f"Creating GraphLattice with diameter {strut_diam} m (~{strut_diam*1000:.1f} mm)")
    trace = pv.GraphLattice(struts, strut_diam, materials.id("black"))

    print("Rendering trace only. Uncomment viz.Export to write STL.")
    viz.Render(trace, materials)
    # viz.Export(trace, materials)


if __name__ == "__main__":
    main()
