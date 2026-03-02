import pyvcad as pv
import pyvcad_rendering as viz

# ---------- Smart Block Prototype 1 - Specifications from Research ----------
# Block dimensions (mm): 20mm Width x 60mm Height x 5mm Depth
block_width = 20.0
block_height = 60.0
block_depth = 5.0
block_dims = pv.Vec3(block_width, block_depth, block_height)  # X, Y, Z in OpenVCAD

# Serpentine conductive trace specifications
trace_width = 0.8      # mm - diameter of conductive path (XY)
trace_height = 1.2     # mm - thickness in Z (3-4 layers at 0.2mm layer height)
trace_spacing = 1.2    # mm - gap between adjacent runs to prevent short circuits

# Prinjection cavity for header pin connection at bottom
cavity_size = pv.Vec3(3.0, 3.0, 5.0)  # mm - sized for 2.54mm header pin
cavity_center = pv.Vec3(0, 0, -block_height/2 + cavity_size.z/2)

# Touch zones are defined by position along the serpentine path:
# - Zone 1 (Low): Bottom 20% (0-12mm height) - Low resistance
# - Zone 2 (Med): Middle 50% (12-42mm height) - Medium resistance  
# - Zone 3 (High): Top 20% (42-60mm height) - High resistance

# Materials: Conductive PLA for sensor, standard PLA for carrier block
mat_conductive = "gray"  # Conductive PLA (Protopasta or Carbon-Black PLA)
mat_structural = "blue"   # Standard PLA (White or PLA+)
# --------------------------------------------------------------------------

materials = pv.default_materials
conductive_mat = materials.id(mat_conductive)
structural_mat = materials.id(mat_structural)


def generate_serpentine_meander():
    """
    Generate a serpentine (snake/meander) pattern that zig-zags horizontally 
    while climbing vertically. The conductive path is continuous.
    
    Design: Horizontal runs alternate left-right, connected by vertical segments.
    This creates a long, thin conductive path for capacitive touch sensing.
    """
    segments = []
    
    # Calculate usable dimensions (leave small margins)
    margin = 2.0
    usable_width = block_width - 2 * margin
    usable_height = block_height - 2 * margin - cavity_size.z  # Room for cavity at bottom
    
    # Calculate number of horizontal runs that fit
    vertical_pitch = trace_width + trace_spacing  # Distance between each horizontal run
    num_runs = int(usable_height / vertical_pitch)
    
    # Starting position at bottom (above cavity)
    start_z = -block_height/2 + cavity_size.z + margin
    
    # Generate the meander pattern
    for i in range(num_runs):
        current_z = start_z + i * vertical_pitch
        
        # Alternate direction for each run (left to right, then right to left)
        if i % 2 == 0:
            # Left to right run
            start_x = -usable_width/2
            end_x = usable_width/2
        else:
            # Right to left run
            start_x = usable_width/2
            end_x = -usable_width/2
        
        # Create horizontal trace segment
        center_x = (start_x + end_x) / 2
        run_length = abs(end_x - start_x)
        horizontal_trace = pv.Cylinder(
            pv.Vec3(center_x, 0, current_z),
            trace_width / 2,
            run_length,
            conductive_mat
        )
        # Rotate to horizontal orientation
        horizontal_trace = pv.Rotate(0, 90, 0, pv.Vec3(center_x, 0, current_z), horizontal_trace)
        segments.append(horizontal_trace)
        
        # Add vertical connector to next run (except for last run)
        if i < num_runs - 1:
            next_z = current_z + vertical_pitch
            connector_z = (current_z + next_z) / 2
            connector_length = vertical_pitch
            
            # Connector at the end of current run
            connector_x = end_x
            vertical_connector = pv.Cylinder(
                pv.Vec3(connector_x, 0, connector_z),
                trace_width / 2,
                connector_length,
                conductive_mat
            )
            segments.append(vertical_connector)
    
    # Add connection trace from cavity to first run
    connection_start_z = cavity_center.z + cavity_size.z/2
    connection_end_z = start_z
    connection_length = connection_end_z - connection_start_z
    if connection_length > 0:
        connection_trace = pv.Cylinder(
            pv.Vec3(-usable_width/2, 0, (connection_start_z + connection_end_z) / 2),
            trace_width / 2,
            connection_length,
            conductive_mat
        )
        segments.append(connection_trace)
    
    return pv.Union(False, segments)


def structural_block():
    """
    Create the carrier block (white PLA) with Prinjection cavity.
    The serpentine pattern will be co-molded / flush with this surface.
    """
    # Main block body
    block = pv.RectPrism(pv.Vec3(0, 0, 0), block_dims, structural_mat)
    
    # Prinjection cavity at bottom center for header pin
    cavity = pv.RectPrism(cavity_center, cavity_size, structural_mat)
    
    # Subtract cavity from block
    return pv.Difference(block, cavity)


def make_smart_block():
    """
    Assemble the complete Smart Block:
    - Structural carrier block (standard PLA)
    - Serpentine conductive sensor (conductive PLA)
    - Prinjection cavity for header pin connection
    
    The result is a co-molded multi-material print where the black
    conductive lines are flush with the white carrier surface.
    """
    structure = structural_block()
    sensor = generate_serpentine_meander()
    
    # Union for multi-material export (not boolean subtraction)
    # The slicer will handle material assignment per volume
    return pv.Union(False, [structure, sensor])


if __name__ == "__main__":
    model = make_smart_block()
    viz.Render(model, materials)
    # When ready to export for multi-material printing:
    # viz.Export(model, materials)