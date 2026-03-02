"""
Variation 2: Render Links Only
Displays only the link path geometry in white (non-conductive structure).
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


def render_links_only(links_pv, verbose: bool = True):
    """Render only the link paths."""
    if not OPENVCAD_AVAILABLE:
        if verbose:
            print("⚠ OpenVCAD not available. Skipping rendering.")
        return
    
    if verbose:
        print("\n" + "=" * 70)
        print("Rendering Link Paths Only")
        print("=" * 70)
    
    materials = pv_cad.default_materials
    
    if verbose:
        print(f"\n[Rendering] Links (non-conductive, white): {links_pv.n_cells} cells")
    
    # Convert to OpenVCAD mesh
    link_mesh = pyvista_to_openvcad_mesh(links_pv, materials.id("white"))
    
    if link_mesh is not None:
        if verbose:
            print("  ✓ Geometry conversion complete")
            print("\n[Rendering] Displaying link paths...")
            print("  - Connection paths: WHITE (structural links between nodes)")
        
        viz.Render(link_mesh, materials)
    else:
        if verbose:
            print("⚠ Failed to convert links to OpenVCAD format")


if __name__ == "__main__":
    print("Generating bunny single-wire sensing model (links only)...\n")
    
    # Generate full model but only render links
    model, geometries = generate_bunny_with_openvcad_integration(
        mesh_file_name="bunny",
        output_dir="../models",
        verbose=True,
        render=False,  # Don't render in main function
    )
    
    # Extract geometries
    surface, nodes, links, resistors = geometries
    
    # Render links only
    render_links_only(links, verbose=True)
