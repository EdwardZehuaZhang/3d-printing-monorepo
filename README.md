Rhino 8 Internal Wire Router
This folder contains a Rhino 8 script-plugin that generates a wire path inside an arbitrary closed solid.

What it does
lets the user select one closed target geometry
lets the user pick start and end terminal connectors on the object first
lets the user pick touch nodes on the object with live preview
trims touch nodes flush to the object boundary
builds an interior routing grid automatically
ranks possible node orders by how close their touch-to-touch resistance steps are to the user-selected target value
tries target-close orders until it finds the nearest routable order that fits the geometry constraints
actively lengthens touch-to-touch routed legs through available interior volume with chained detours when the direct route is too short for the requested resistance target
routes one continuous conductive path from start to end through all touch nodes in that selected order
adds separate conductive path and conductive pathway node solids back into Rhino

This version is intended for:
closed meshes
closed Breps / polysurfaces / extrusions that can be meshed cleanly
SubD objects that can be converted to a closed mesh
It does not yet do surface-only routing on open geometry.

Files
Commands/GenerateInternalWire.py — Rhino command entry point
Libraries/wire_router/core.py — pure Python A* grid router
Libraries/wire_router/rhino_router.py — RhinoCommon integration

Quick Start In Rhino 8
Prerequisites:
Rhino 8 installed
this repository available on your computer
Try the plugin locally on your machine:

Open Rhino 8.
Run ScriptEditor.
Choose File > Create Project.
Fill in the required project metadata such as author, then save the project.
In the Script Editor project tree, add Commands/GenerateInternalWire.py into the Commands folder.
In the Script Editor project tree, add the folder Libraries/wire_router as a Python library under Libraries.
Save the project.
Use Script Editor run/debug for quick iteration while editing the source.
When you want the command available from the Rhino command line, publish the project with Publish Project > Build Package.
After publish completes, install the generated .rhp with PlugInManager > Install....
If Rhino asks, load the plugin.
Run GenerateInternalWire in Rhino.

Workflow inside Rhino:
Start GenerateInternalWire.
Select one closed solid, closed mesh, extrusion, or SubD.
Enter the conductive pathway node sphere diameter in millimeters. Press Enter to accept the default 10.0 mm finger-sized conductive touch sensor node.
Enter the conductive path diameter in millimeters. Press Enter to accept the default 0.5 mm single printable line.
Enter the desired touch-to-touch resistance between successive nodes in kohm. Press Enter to accept the recommended default based on the selected conductive path diameter. With the default 0.5 mm single printable line, the suggested target is 200.0 kohm for stronger noise separation.
Pick the Start terminal.
Set that terminal to Flush or Protrude.
Pick the End terminal.
Set that terminal to Flush or Protrude.
Pick conductive pathway nodes on the surface.
Press Enter when all nodes are placed.
Inspect the generated conductive path and conductive nodes.

Generated geometry
The command currently creates three outputs:
a polyline centerline named GenerateInternalWire_Centerline
conductive path solid(s) named GenerateInternalWire_ConductivePath
conductive pathway node solid(s) named GenerateInternalWire_ConductivePathwayNodes
The selected object is used as the host body for routing and node placement, but it is not modified automatically. The generated solids are separate objects that can be used for downstream boolean operations or fabrication workflows.

Electrical estimate
The command prints an estimated conductive resistance range using ProtoPasta Conductive PLA, scaled from the published 2.0-3.5 kohm per 10 cm value for 1.75 mm filament and the user-selected conductive path diameter.
