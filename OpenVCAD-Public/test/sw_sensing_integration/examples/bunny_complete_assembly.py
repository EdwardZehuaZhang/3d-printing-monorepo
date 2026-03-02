"""
Variation 4: Render Body + All Components
Displays the complete model with base body (white) and all sensing components
(nodes and resistors in blue, links in white).
"""

import sys
import os

# Add the sw_sensing_integration directory to the path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from bunny_single_openvcad import generate_bumny_with_openvcad_integration

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


def render_body_and_components(
    surface_pv, nodes_pv, links_pv, resistors_pv, verbose: bool = True
):
    """Render body with all components."""
    if not OPENVCAD_AVAILABLE:
        if verbose:
            print("⚠ OpenVCAD not available. Skipping rendering.")
        return
    
    if verbose:
        print("\n" + "=" * 70)
        print("Rendering Body + All Components")
        print("=" * 70)
    
    materials = pv_cad.default_materials
    
    if verbose:
        print("\n[Rendering] Converting geometries to OpenVCAD format...")
    
    # Convert geometries - conductive in blue, non-conductive in white
    resistor_mesh = pyvista_to_openvcad_mesh(resistors_pv, materials.id("blue"))
    node_mesh = pyvista_to_openvcad_mesh(nodes_pv, materials.id("blue"))
    link_mesh = pyvista_to_openvcad_mesh(links_pv, materials.id("white"))
    body_mesh = pyvista_to_openvcad_mesh(surface_pv, materials.id("white"))
    
    if verbose:
        print("  ✓ Geometry conversion complete")
        print("\n[Rendering] Building composition...")
    
    # Build composition with all components
    components = []
    if body_mesh is not None:
        components.append(body_mesh)
        if verbose:
            print(f"  ✓ Body (exterior, white): {surface_pv.n_cells} cells")
    
    if link_mesh is not None:
        components.append(link_mesh)
        if verbose:
            print(f"  ✓ Links (non-conductive, white): {links_pv.n_cells} cells")
    
    if node_mesh is not None:
        components.append(node_mesh)
        if verbose:
            print(f"  ✓ Nodes (conductive, blue): {nodes_pv.n_cells} cells")
    
    if resistor_mesh is not None:
        components.append(resistor_mesh)
        if verbose:
            print(f"  ✓ Resistors (conductive, blue): {resistors_pv.n_cells} cells")
    
    if len(components) > 0:
        # Create union of all components
        root = pv_cad.Union(False, components)
        
        if verbose:
            print("\n[Rendering] Displaying complete model...")
            print("  - Base body: WHITE")
            print("  - Connection links: WHITE (structural)")
            print("  - Touch nodes: BLUE (conductive contact)")
            print("  - Resistor traces: BLUE (conductive sensing)")
        
        # Render
        viz.Render(root, materials)
    else:
        if verbose:
            print("⚠ No components to render")


if __name__ == "__main__":
    print("Generating bunny single-wire sensing model (complete assembly)...\n")
    
    # Generate full model
    model, geometries = generate_bunny_with_openvcad_integration(
        mesh_file_name="bunny",
        output_dir="../models",
        verbose=True,
        render=False,  # Don't render in main function
    )
    
    # Extract geometries
    surface, nodes, links, resistors = geometries
    
    # Render complete assembly
    render_body_and_components(surface, nodes, links, resistors, verbose=True)
