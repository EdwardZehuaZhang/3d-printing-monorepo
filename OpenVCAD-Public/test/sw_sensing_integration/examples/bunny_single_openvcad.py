"""
Refactored Example: Single-Wire Sensing Model Generation with OpenVCAD Integration

This script demonstrates the improved workflow that generates a single integrated
3D model with proper material assignments, eliminating the need for manual alignment
in PrusaSlicer.

Key improvements over original approach:
1. Single model output instead of 4 separate files
2. Material information preserved in the model
3. Proper composition using implicit geometry
4. No manual alignment required in slicing software
5. Direct import to multi-material slicing

Usage:
    python bunny_single_openvcad.py

Output:
    - bunny_integrated_material_map.json: Material assignment metadata
    - (Future) bunny_integrated.3mf: Multi-material model for slicing
"""

import json
import sys
import os
import numpy as np
import pyvista as pv

# Add the sw_sensing_integration directory to the path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

# Import sw_sensing components
import sw_sensing.geom_utils as gu
from sw_sensing.path_finding import graph_based_path_finding
from sw_sensing.geometry import (
    node_order_selection,
    generate_all_geometries,
    default_args,
)
from sw_sensing.single_wiring_optimization import optimize_resistances

# Import new OpenVCAD integration
from sw_sensing.openvcad_integration import (
    SWSGeometryInput,
    MaterialConfig,
    create_integrated_model_from_sw_sensing,
)

# Import OpenVCAD for rendering
try:
    import pyvcad as pv_cad
    import pyvcad_rendering as viz
    OPENVCAD_AVAILABLE = True
except ImportError:
    OPENVCAD_AVAILABLE = False


def pyvista_to_openvcad_mesh(pv_mesh, material_id):
    """
    Convert PyVista mesh to OpenVCAD Mesh.
    
    Args:
        pv_mesh: PyVista PolyData mesh
        material_id: Material ID string
        
    Returns:
        OpenVCAD Mesh object
    """
    if not OPENVCAD_AVAILABLE:
        return None
    
    # Export PyVista mesh to temporary STL and reload via OpenVCAD
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


def render_integrated_model(
    surface_pv,
    nodes_pv,
    links_pv,
    resistors_pv,
    mesh_file_name: str = "bunny",
    output_dir: str = "../models",
    verbose: bool = True,
):
    """
    Render integrated single-wire sensing model with OpenVCAD.
    Colors: conductive (blue), non-conductive (white), exterior (light gray).
    
    Args:
        surface_pv: PyVista mesh of the base body
        nodes_pv: PyVista mesh of touch nodes
        links_pv: PyVista mesh of link paths
        resistors_pv: PyVista mesh of resistor traces
        mesh_file_name: Base model name
        output_dir: Output directory
        verbose: Print progress information
    """
    if not OPENVCAD_AVAILABLE:
        if verbose:
            print("⚠ OpenVCAD not available. Skipping rendering.")
        return
    
    if verbose:
        print("\n" + "=" * 70)
        print("Rendering Integrated Single-Wire Sensing Model")
        print("=" * 70)
    
    materials = pv_cad.default_materials
    
    if verbose:
        print("\n[Rendering] Converting geometries to OpenVCAD format...")
    
    # Convert geometries to OpenVCAD meshes
    # Conductive components (blue): resistors and nodes
    resistor_mesh = pyvista_to_openvcad_mesh(resistors_pv, materials.id("blue"))
    node_mesh = pyvista_to_openvcad_mesh(nodes_pv, materials.id("blue"))
    
    # Non-conductive components (white): links and body
    link_mesh = pyvista_to_openvcad_mesh(links_pv, materials.id("white"))
    body_mesh = pyvista_to_openvcad_mesh(surface_pv, materials.id("white"))
    
    if verbose:
        print("  ✓ Geometry conversion complete")
        print("\n[Rendering] Building composition...")
    
    # Build composition: body with components
    components = []
    if node_mesh is not None:
        components.append(node_mesh)
        if verbose:
            print(f"  ✓ Nodes (conductive, blue): {nodes_pv.n_cells} cells")
    
    if link_mesh is not None:
        components.append(link_mesh)
        if verbose:
            print(f"  ✓ Links (non-conductive, white): {links_pv.n_cells} cells")
    
    if resistor_mesh is not None:
        components.append(resistor_mesh)
        if verbose:
            print(f"  ✓ Resistors (conductive, blue): {resistors_pv.n_cells} cells")
    
    if body_mesh is not None:
        components.append(body_mesh)
        if verbose:
            print(f"  ✓ Body (exterior, white): {surface_pv.n_cells} cells")
    
    if len(components) > 0:
        # Create union of all components
        root = pv_cad.Union(False, components)
        
        if verbose:
            print("\n[Rendering] Displaying integrated model...")
            print("  - Conductive traces: BLUE (resistors, nodes)")
            print("  - Non-conductive structure: WHITE (body, links)")
        
        # Render
        viz.Render(root, materials)
    else:
        if verbose:
            print("⚠ No components to render")


def generate_bunny_with_openvcad_integration(
    mesh_file_name: str = "bunny",
    output_dir: str = "../models",
    node_radius: float = 6.0,
    min_dist_from_surface: float = 3.0,
    voxel_resolution: int = 200,
    verbose: bool = True,
    render: bool = True,
):
    """
    Generate single-wire sensing model with integrated OpenVCAD output.
    
    Args:
        mesh_file_name: Base model name (e.g., "bunny")
        output_dir: Output directory
        node_radius: Radius of touch nodes
        min_dist_from_surface: Minimum distance of wire paths from surface
        voxel_resolution: Voxel grid resolution for pathfinding
        verbose: Print progress information
        render: Whether to render the model using OpenVCAD
        
    Returns:
        tuple: (SWSOpenVCADBuilder, geometries) where geometries = (surface, nodes, links, resistors)

    """
    
    if verbose:
        print("=" * 70)
        print("Single-Wire Sensing Model Generation with OpenVCAD Integration")
        print("=" * 70)
    
    # ========================================================================
    # Step 1: Node Preparation
    # ========================================================================
    if verbose:
        print("\n[1/4] Node Preparation")
        print("-" * 70)
    
    surface = pv.get_reader(f"{output_dir}/{mesh_file_name}.stl").read()
    with open(f"{output_dir}/selection_{mesh_file_name}.json", "r") as f:
        set_of_seleceted_vertex_indices = json.load(f)
    
    node_positions = gu.selected_polygons_to_node_positions(
        set_of_seleceted_vertex_indices, mesh=surface
    )
    nodes = np.arange(len(node_positions))
    
    # Define connection nodes
    connection_start_node = nodes[-2]
    connection_end_node = None
    nodes = nodes[:-1]
    node_positions = node_positions[:-1]
    
    # Select node order (can be optimized)
    node_order = node_order_selection(
        initial_order=nodes,
        method="asis",
        start_node=connection_start_node,
        end_node=connection_end_node,
    )
    # Override with specific order if needed
    node_order = np.array([5, 4, 3, 1, 2, 0])
    
    if verbose:
        print(f"  ✓ Loaded mesh: {mesh_file_name}.stl")
        print(f"  ✓ {len(node_positions)} touch nodes identified")
        print(f"  ✓ Node order: {node_order}")
    
    # ========================================================================
    # Step 2: Link Path Finding
    # ========================================================================
    if verbose:
        print("\n[2/4] Link Path Finding")
        print("-" * 70)
    
    voxels = gu.surface_to_voxels(surface, resolution=voxel_resolution)
    inner_voxels = voxels.clip_scalar(
        scalars="implicit_distance", value=-min_dist_from_surface
    )
    voxel_graph = gu.pointset_to_graph(inner_voxels, n_neighbors=10)
    
    set_of_link_path_positions = graph_based_path_finding(
        node_positions[node_order],
        g=voxel_graph,
        min_space=5,
        min_space_violation_penalty=300,
    )
    
    if verbose:
        print(f"  ✓ Voxel grid resolution: {voxel_resolution}")
        print(f"  ✓ {len(set_of_link_path_positions)} link paths found")
        for i, path in enumerate(set_of_link_path_positions):
            print(f"    - Path {i}: {len(path)} waypoints")
    
    # ========================================================================
    # Step 3: Geometry Preparation
    # ========================================================================
    if verbose:
        print("\n[3/4] Geometry Preparation")
        print("-" * 70)
    
    (
        combined_node_object,
        combined_link_object,
        combined_resistor_object,
    ) = generate_all_geometries(
        surface=surface,
        node_order=node_order,
        node_positions=node_positions,
        connection_start_node=connection_start_node,
        connection_end_node=connection_end_node,
        set_of_link_path_positions=set_of_link_path_positions,
        save_file_basename=f"{output_dir}/{mesh_file_name}",
        node_kwargs={
            **default_args(generate_all_geometries)["node_kwargs"],
            "radius": node_radius,
            "surface_voxels": voxels,
            "padding_from_surface": -0.5,
        },
        link_kwargs={
            **default_args(generate_all_geometries)["link_kwargs"],
            "radius": [1.5] + [2.5] * (len(set_of_link_path_positions) - 1),
        },
        resistor_trace_fill_kwargs={
            **default_args(generate_all_geometries)["resistor_trace_fill_kwargs"],
            "aiming_resistance": 400e3,
            "learning_rate": 0.1,
            "max_iterations": 300,
            "no_overlap_margin": 2.0,
            "max_fill_area_radius": [1.0]
            + [2.0] * (len(set_of_link_path_positions) - 1),
            "min_dy": 1.2,
            "min_dz": 1.2,
        },
        resistor_trace_kwargs={"horizontal_width": 0.8, "vertical_width": 1.2},
        node_to_resistor_trace_kwargs={
            "trace_horizontal_width": 0.8,
            "trace_vertical_width": 1.2,
        },
    )
    
    if verbose:
        print(f"  ✓ Nodes geometry prepared: {combined_node_object.n_cells} cells")
        print(f"  ✓ Links geometry prepared: {combined_link_object.n_cells} cells")
        print(f"  ✓ Resistors geometry prepared: {combined_resistor_object.n_cells} cells")
    
    # ========================================================================
    # Step 4: Resistance Optimization
    # ========================================================================
    if verbose:
        print("\n[4/4] Resistance Optimization")
        print("-" * 70)
    
    n_nodes = len(nodes)
    voltage_thres = 2.5
    wire_r, trace_r, score = optimize_resistances(
        n_nodes=n_nodes - 1,
        wire_resistance_candidates=np.arange(0.1e6, 10e6, 200e3),
        trace_resistance_candidates=np.arange(100e3, 300e3 + 50e3, 50e3) * 0.23,
    )
    
    if verbose:
        print(f"  ✓ Optimal wire resistance: {wire_r:.2e} Ω")
        print(f"  ✓ Optimal trace resistance: {trace_r:.2e} Ω")
        print(f"  ✓ Optimization score: {score}")
    
    # ========================================================================
    # Step 5: Build Integrated OpenVCAD Model
    # ========================================================================
    if verbose:
        print("\n[5/5] Building Integrated OpenVCAD Model")
        print("-" * 70)
    
    # Prepare geometry input
    geometry_input = SWSGeometryInput(
        base_mesh_pv=surface,
        node_positions=node_positions,
        node_radius=node_radius,
        link_paths=set_of_link_path_positions,
        link_radii=[1.5] + [2.5] * (len(set_of_link_path_positions) - 1),
        resistor_meshes_pv=[combined_resistor_object],
        node_order=node_order,
    )
    
    # Configure materials
    materials = MaterialConfig(
        conductive_id="conductive_filament",
        nonconductive_id="standard_filament",
        exterior_id="exterior_structure",
    )
    
    # Build integrated model
    model_builder = create_integrated_model_from_sw_sensing(
        geometry_input=geometry_input,
        materials=materials,
        verbose=verbose,
    )
    
    # ========================================================================
    # Step 6: Export
    # ========================================================================
    if verbose:
        print("\n[6/6] Exporting Model")
        print("-" * 70)
    
    output_basename = f"{output_dir}/{mesh_file_name}_integrated"
    model_builder.export_to_3mf(f"{output_basename}.3mf")
    
    if verbose:
        print(f"\n✓ Model generation complete!")
        print(f"  Output: {output_basename}.*")
        print(f"\nNEXT STEPS:")
        print(f"  1. Open {output_basename}.3mf in PrusaSlicer or similar")
        print(f"  2. Assign materials to each component:")
        print(f"     - Non-conductive regions: standard filament")
        print(f"     - Conductive regions: conductive filament")
        print(f"  3. Configure print settings")
        print(f"  4. Export G-code for multi-material 3D printing")
    
    # ========================================================================
    # Step 7: Render (Optional)
    # ========================================================================
    if render:
        render_integrated_model(
            surface_pv=surface,
            nodes_pv=combined_node_object,
            links_pv=combined_link_object,
            resistors_pv=combined_resistor_object,
            mesh_file_name=mesh_file_name,
            output_dir=output_dir,
            verbose=verbose,
        )
    
    return model_builder, (surface, combined_node_object, combined_link_object, combined_resistor_object)


if __name__ == "__main__":
    # Generate model with rendering
    model, geometries = generate_bunny_with_openvcad_integration(
        mesh_file_name="bunny",
        output_dir="../models",
        verbose=True,
        render=True,
    )
    
    # Optional: Visualize component summary
    print("\n" + model.build_summary())
