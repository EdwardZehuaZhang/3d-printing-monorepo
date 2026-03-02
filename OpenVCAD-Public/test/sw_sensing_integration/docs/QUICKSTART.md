# Quick Start Guide: OpenVCAD Integration

## In 5 Minutes

### 1. Run the Generation Script

```bash
cd examples
python bunny_single_openvcad.py
```

**Output:**
```
======================================================================
Single-Wire Sensing Model Generation with OpenVCAD Integration
======================================================================

[1/4] Node Preparation
  ✓ Loaded mesh: bunny.stl
  ✓ 6 touch nodes identified
  ✓ Node order: [5 4 3 1 2 0]

[2/4] Link Path Finding
  ✓ Voxel grid resolution: 200
  ✓ 6 link paths found
    - Path 0: 45 waypoints
    - Path 1: 38 waypoints
    ...

[3/4] Geometry Preparation
  ✓ Nodes geometry prepared: 12456 cells
  ✓ Links geometry prepared: 8932 cells
  ✓ Resistors geometry prepared: 45678 cells

[4/4] Resistance Optimization
  ✓ Optimal wire resistance: 7.00e+05 Ω
  ✓ Optimal trace resistance: 2.07e+04 Ω
  ✓ Optimization score: 0.85

[5/5] Building Integrated OpenVCAD Model
  Building integrated OpenVCAD model from sw_sensing outputs...
  ✓ Base mesh added
  ✓ 6 nodes added
  ✓ 6 link paths added
  ✓ 1 resistor traces added

Model export metadata saved to: ../models/bunny_integrated_material_map.json

Component Summary:
  base_mesh: {'type': 'mesh', 'material': 'non-conductive'}
  nodes: {'count': 6, 'material': 'conductive', ...}
  links: {'count': 6, 'material': 'non-conductive', ...}
  resistors: {'count': 1, 'material': 'conductive', ...}

[6/6] Exporting Model
✓ Model generation complete!
  Output: ../models/bunny_integrated.*

NEXT STEPS:
  1. Open ../models/bunny_integrated.3mf in PrusaSlicer or similar
  2. Assign materials to each component:
     - Non-conductive regions: standard filament
     - Conductive regions: conductive filament
  3. Configure print settings
  4. Export G-code for multi-material 3D printing
```

### 2. Import into PrusaSlicer

1. Open PrusaSlicer
2. File → Open → Select `bunny_integrated.3mf`
3. Model loads with all components pre-aligned
4. No manual rotation/positioning needed!

### 3. Assign Materials

In the left panel:
- Click on "conductive_filament" parts → Assign conductive material
- Click on "standard_filament" parts → Assign regular material

### 4. Print

Configure your printer's multi-material settings and print!

---

## Integration into Existing Scripts

### Before (Original)
```python
(node_obj, link_obj, resistor_obj) = generate_all_geometries(...)
node_obj.save("model_node.stl")
link_obj.save("model_link.stl")
resistor_obj.save("model_resistor.stl")
surface.save("model.stl")
# Then manually align 4 files in PrusaSlicer
```

### After (OpenVCAD)
```python
from sw_sensing.openvcad_integration import SWSGeometryInput, create_integrated_model_from_sw_sensing

(node_obj, link_obj, resistor_obj) = generate_all_geometries(...)

geometry = SWSGeometryInput(
    base_mesh_pv=surface,
    node_positions=node_positions,
    node_radius=6.0,
    link_paths=set_of_link_path_positions,
    link_radii=[...],
    resistor_meshes_pv=[resistor_obj],
    node_order=node_order,
)

model = create_integrated_model_from_sw_sensing(geometry)
model.export_to_3mf("model_integrated.3mf")
# Single file ready for import!
```

---

## API Cheat Sheet

### Create Input Container
```python
from sw_sensing.openvcad_integration import SWSGeometryInput

geo = SWSGeometryInput(
    base_mesh_pv=surface,
    node_positions=np.array([[x1, y1, z1], ...]),  # Shape (n, 3)
    node_radius=6.0,
    link_paths=[path1, path2, ...],                 # List of arrays
    link_radii=[1.5, 2.5, 2.5],
    resistor_meshes_pv=[resistor_pyvista_mesh],
    node_order=np.array([5, 4, 3, 1, 2, 0])
)
```

### Configure Materials
```python
from sw_sensing.openvcad_integration import MaterialConfig

materials = MaterialConfig(
    conductive_id="conductive_filament",
    nonconductive_id="standard_filament"
)
```

### Build & Export
```python
from sw_sensing.openvcad_integration import create_integrated_model_from_sw_sensing

model = create_integrated_model_from_sw_sensing(
    geometry_input=geo,
    materials=materials,
    verbose=True  # Shows progress
)

# Inspect model
print(model.build_summary())
summary = model.get_component_summary()

# Export
model.export_to_3mf("output/model.3mf")
```

---

## What Gets Generated?

### Output Files

1. **`model_integrated.3mf`** - Ready for slicing software
   - Single integrated model
   - Material assignments preserved
   - No alignment needed

2. **`model_integrated_material_map.json`** - Material reference
   ```json
   {
     "format": "3mf",
     "components": {
       "base_mesh": {"type": "mesh", "material": "non-conductive"},
       "nodes": {"count": 6, "material": "conductive"},
       "links": {"count": 6, "material": "non-conductive"},
       "resistors": {"count": 1, "material": "conductive"}
     },
     "materials": {
       "conductive": "conductive_filament",
       "nonconductive": "standard_filament"
     }
   }
   ```

---

## Materials

All materials are identified by strings:

| ID | Purpose | Example |
|----|---------|---------|
| `conductive_filament` | Touch nodes & resistor traces | ColorFabb Electrify, Protopasta |
| `standard_filament` | Exterior mesh & conduits | PLA, PETG, Nylon |

You define these in `MaterialConfig` - they match whatever you have available.

---

## Comparison: Before & After

### Before (Original Method)

```
bunny_single.py
    ↓
Generate 4 meshes (node, link, resistor, base)
    ↓
Save 4 STL files
    ↓
Open PrusaSlicer
    ↓
MANUALLY import 4 files
    ↓
MANUALLY rotate/align each
    ↓
MANUALLY assign materials to each
    ↓
Ready to slice
    (time: ~30 minutes)
```

### After (OpenVCAD Integration)

```
bunny_single_openvcad.py
    ↓
Generate 4 geometries (unchanged)
    ↓
Build integrated OpenVCAD model
    ↓
Export single 3MF file
    ↓
Open PrusaSlicer
    ↓
IMPORT single file (auto-aligned!)
    ↓
Assign materials
    ↓
Ready to slice
    (time: ~5 minutes)
```

---

## Troubleshooting

### Q: What if I don't have OpenVCAD installed?

**A:** The module degrades gracefully:
- Full model building works
- Export to 3MF creates metadata (reference file)
- Compilation requires OpenVCAD (from MacCurdy Lab)

### Q: Can I use this with non-bunny models?

**A:** Yes! Apply the same pattern to any model with:
- An STL file
- Selection JSON with touchpoint indices
- Refactored to use `bunny_single_openvcad.py` pattern

### Q: What about the original separate-file method?

**A:** Still available! `bunny_single.py` unchanged.
- Use either method depending on your workflow
- Original useful for inspection/debugging
- New method better for manufacturing

### Q: How do I debug issues?

**A:** Use verbose mode:
```python
model = create_integrated_model_from_sw_sensing(geo, verbose=True)
summary = model.get_component_summary()
print(model.build_summary())
```

---

## Next Steps

1. **Try the example:**
   ```bash
   python examples/bunny_single_openvcad.py
   ```

2. **Import to PrusaSlicer:**
   - Check that all components are aligned
   - Verify material assignments

3. **Adapt to your models:**
   - Use `bunny_single_openvcad.py` as template
   - Change model name and file paths
   - Run your model!

4. **Check the docs:**
   - [REFACTORING_SUMMARY.md](REFACTORING_SUMMARY.md) - Full documentation
   - [sw_sensing/openvcad_integration.py](sw_sensing/openvcad_integration.py) - API reference

---

## Questions?

See the full implementation:
- Main module: `sw_sensing/openvcad_integration.py`
- Example: `examples/bunny_single_openvcad.py`
- Documentation: `REFACTORING_SUMMARY.md`
