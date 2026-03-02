"""
Legacy Body + Path: path from watertight PLY (matches body scale).
The bundled OBJ and PLY are scaled differently (~100x), so sampling the PLY
avoids a tiny trace that disappears inside the body.
"""
import pyvcad as pv
import pyvcad_rendering as viz
import numpy as np
from pathlib import Path
from scipy.spatial import distance_matrix


def load_mesh_vertices(path: Path) -> np.ndarray:
    """Load vertices from OBJ via manual parser; otherwise via trimesh (for PLY)."""
    if path.suffix.lower() == ".obj":
        verts = []
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("v "):
                    _, x, y, z = line.strip().split()
                    verts.append([float(x), float(y), float(z)])
        return np.array(verts)
    import trimesh  # lazy import to keep legacy feel

    mesh = trimesh.load_mesh(str(path), process=False)
    return np.asarray(mesh.vertices)


def sample_surface_points(vertices: np.ndarray, num_points=500) -> np.ndarray:
    idx = np.random.choice(len(vertices), size=min(num_points, len(vertices)), replace=False)
    return vertices[idx]


def build_knn_graph(points: np.ndarray, k=5):
    dist = distance_matrix(points, points)
    edges = []
    for i in range(len(points)):
        nearest = np.argsort(dist[i])[: k + 1]
        for j in nearest[1:]:
            if i < j:
                edges.append((i, j, dist[i, j]))
    return edges


def build_spanning_tree(points: np.ndarray, edges):
    edges_sorted = sorted(edges, key=lambda x: x[2])
    parent = list(range(len(points)))

    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py
            return True
        return False

    mst_edges = []
    for i, j, w in edges_sorted:
        if union(i, j):
            mst_edges.append((i, j))
        if len(mst_edges) == len(points) - 1:
            break
    return mst_edges


def mst_to_path(mst_edges, n):
    adj = [[] for _ in range(n)]
    for i, j in mst_edges:
        adj[i].append(j)
        adj[j].append(i)
    visited = [False] * n
    path = []

    def dfs(node):
        visited[node] = True
        path.append(node)
        for nbr in adj[node]:
            if not visited[nbr]:
                dfs(nbr)
                path.append(node)

    dfs(0)
    return path


def generate_curve_points(mesh_path: Path, num_points=500, k_neighbors=5):
    print(f"Loading mesh vertices from: {mesh_path}")
    verts = load_mesh_vertices(mesh_path)
    print(f"Loaded {len(verts)} vertices")
    print(f"Sampling {num_points} surface points...")
    pts = sample_surface_points(verts, num_points)
    print(f"Building k-NN graph (k={k_neighbors})...")
    edges = build_knn_graph(pts, k_neighbors)
    print(f"Generated {len(edges)} edges")
    print("Building minimum spanning tree...")
    mst = build_spanning_tree(pts, edges)
    print(f"MST has {len(mst)} edges")
    print("Converting MST to continuous path...")
    order = mst_to_path(mst, len(pts))
    print(f"Generated path with {len(order)} points")
    return [pv.Vec3(*pts[i]) for i in order]


def main():
    script_dir = Path(__file__).parent
    # Base models live one directory up from the legacy scripts
    obj_file = script_dir.parent / "base-models" / "bunny.obj"
    ply_file = script_dir.parent / "base-models" / "bunny_watertight.ply"
    materials = pv.default_materials

    if not obj_file.exists():
        raise FileNotFoundError(f"Expected OBJ at {obj_file}")
    if not ply_file.exists():
        raise FileNotFoundError(f"Expected watertight PLY at {ply_file}")

    # Use the watertight PLY for both body and path to keep scales aligned.
    curve_pts = generate_curve_points(ply_file, num_points=500, k_neighbors=5)
    struts = [(curve_pts[i], curve_pts[i + 1]) for i in range(len(curve_pts) - 1)]
    print(f"Creating GraphLattice with {len(struts)} struts...")
    # The watertight PLY is ~15 units across; use a thicker strut for visibility.
    trace = pv.GraphLattice(struts, 0.1, materials.id("blue"))

    print(f"Loading watertight body: {ply_file}")
    body = pv.Mesh(str(ply_file), materials.id("white"))
    minus = pv.Difference(body, trace)
    root = pv.Union(False, [minus, trace])

    print("Rendering legacy body + trace. Uncomment viz.Export to export.")
    viz.Render(root, materials)
    # viz.Export(root, materials)


if __name__ == "__main__":
    main()
