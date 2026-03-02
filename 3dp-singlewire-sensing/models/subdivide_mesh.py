import pyvista as pv

if __name__ == "__main__":
    mesh_file_name = "power"
    surface = pv.get_reader(f"{mesh_file_name}.stl").read()
    surface.subdivide_adaptive().save(f"{mesh_file_name}_subdivided.stl")
