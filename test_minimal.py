#!/usr/bin/env python3
import sys
sys.path.insert(0, '/Users/gel/Desktop/Github/3d-printing-monorepo/OpenVCAD-Public/test/sw_sensing_integration')

print("Step 1: Import bunny_single_openvcad...")
try:
    from examples.bunny_single_openvcad import generate_bunny_with_openvcad_integration
    print("✓ Import successful")
except Exception as e:
    print(f"✗ Import failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\nStep 2: Generating bunny model...")
try:
    model, geometries = generate_bunny_with_openvcad_integration(
        mesh_file_name="bunny",
        output_dir="/Users/gel/Desktop/Github/3d-printing-monorepo/OpenVCAD-Public/test/sw_sensing_integration/examples/../models",
        verbose=True,
        render=False,
    )
    print("✓ Model generation successful")
except Exception as e:
    print(f"✗ Model generation failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n✓ All tests passed!")
