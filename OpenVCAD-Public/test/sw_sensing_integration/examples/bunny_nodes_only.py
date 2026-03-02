"""
Variation 1: Render Nodes Only
Displays only the touch node geometry in blue (conductive material).
"""

import sys
import os

# Add the sw_sensing_integration directory to the path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from bunny_single_openvcad import generate_bunny_with_openvcad_integration

try:
    import pyvcad as pv_cad
    import pyvcad_rendering as viz
    OPENVCAD_AVAILABLE = True
except ImportError:
    OPENVCAD_AVAILABLE = False


def pyvista_to_openvcad_mesh(pv_mesh, material_id):
    """Convert PyVista mesh to OpenVCAD Mesh."""
    if not OPENVCAD_AVAILABLE:
        return None
    
    import tempfile
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
        temp_path = f.name
    
    try:
        pv_mesh.save(temp_path)
        mesh = pv_cad.Mesh(temp_path, material_id)
        return mesh
    finally:
        import os
        if os.path.exists(temp_path):
            os.remove(temp_path)


def render_nodes_only(nodes_pv, verbose: bool = True):
    """Render only the touch nodes."""
    if not OPENVCAD_AVAILABLE:
        if verbose:
            print("⚠ OpenVCAD not available. Skipping rendering.")
        return
    
    if verbose:
        print("\n" + "=" * 70)
        print("Rendering Touch Nodes Only")
        print("=" * 70)
    
    materials = pv_cad.default_materials
    
    if verbose:
        print(f"\n[Rendering] Nodes (conductive, blue): {nodes_pv.n_cells} cells")
    
    # Convert to OpenVCAD mesh
    node_mesh = pyvista_to_openvcad_mesh(nodes_pv, materials.id("blue"))
    
    if node_mesh is not None:
        if verbose:
            print("  ✓ Geometry conversion complete")
            print("\n[Rendering] Displaying nodes...")
            print("  - Touch nodes: BLUE (conductive contact points)")
        
        viz.Render(node_mesh, materials)
    else:
        if verbose:
            print("⚠ Failed to convert nodes to OpenVCAD format")


if __name__ == "__main__":
    print("Generating bunny single-wire sensing model (nodes only)...\n")
    
    # Generate full model but only render nodes
    model, geometries = generate_bunny_with_openvcad_integration(
        mesh_file_name="bunny",
        output_dir="../models",
        verbose=True,
        render=False,  # Don't render in main function
    )
    
    # Extract geometries
    surface, nodes, links, resistors = geometries
    
    # Render nodes only
    render_nodes_only(nodes, verbose=True)
