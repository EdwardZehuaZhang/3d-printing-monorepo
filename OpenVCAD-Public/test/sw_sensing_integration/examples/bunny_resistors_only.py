"""
Variation 3: Render Resistors Only
Displays only the resistor trace geometry in blue (conductive traces).
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


def render_resistors_only(resistors_pv, verbose: bool = True, display: bool = True):
    """Render only the resistor traces."""
    if not OPENVCAD_AVAILABLE:
        if verbose:
            print("⚠ OpenVCAD not available. Skipping rendering.")
        return
    
    if verbose:
        print("\n" + "=" * 70)
        print("Rendering Resistor Traces Only")
        print("=" * 70)
    
    materials = pv_cad.default_materials
    
    if verbose:
        print(f"\n[Rendering] Resistors (conductive, blue): {resistors_pv.n_cells} cells")
    
    # Convert to OpenVCAD mesh
    resistor_mesh = pyvista_to_openvcad_mesh(resistors_pv, materials.id("blue"))
    
    if resistor_mesh is not None:
        if verbose:
            print("  ✓ Geometry conversion complete")
            if display:
                print("\n[Rendering] Displaying resistor traces...")
                print("  - Resistor traces: BLUE (conductive resistance pathways)")
        
        if display:
            viz.Render(resistor_mesh, materials)
        elif verbose:
            print("  (Display skipped - headless mode)")
    else:
        if verbose:
            print("⚠ Failed to convert resistors to OpenVCAD format")


if __name__ == "__main__":
    print("Generating bunny single-wire sensing model (resistors only)...\n")
    
    # Generate full model but only render resistors
    model, geometries = generate_bunny_with_openvcad_integration(
        mesh_file_name="bunny",
        output_dir="/Users/gel/Desktop/Github/3d-printing-monorepo/3dp-singlewire-sensing/models",
        verbose=True,
        render=False,  # Don't render in main function
    )
    
    # Extract geometries
    surface, nodes, links, resistors = geometries
    
    # Render resistors only (display=False for headless execution)
    render_resistors_only(resistors, verbose=True, display=False)
