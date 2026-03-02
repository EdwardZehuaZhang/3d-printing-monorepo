from distutils.core import setup

setup(
    name="sw-sensing",
    version=0.1,
    package_dir={"": "."},
    install_requires=[
        "lcapy",
        "numpy",
        "scipy",
        "scikit-learn",
        "vtk==9.3.0",
        "pyvista==0.43.3",
        "sensing-network",
    ],
    py_modules=[
        "sw_sensing.geom_utils",
        "sw_sensing.path_finding",
        "sw_sensing.geometry",
        "sw_sensing.single_wiring_optimization",
    ],
)
