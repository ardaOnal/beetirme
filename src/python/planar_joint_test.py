from pydrake.all import (DiagramBuilder, StartMeshcat, LoadModelDirectivesFromString, 
                         AddMultibodyPlantSceneGraph, Parser, ProcessModelDirectives, Simulator, MeshcatVisualizer)

from manipulation.utils import AddPackagePaths

import os

MAX_TIME = 300

# Start the visualizer.
meshcat = StartMeshcat()

meshcat.Delete()
builder = DiagramBuilder()
model_directives = """
directives:
- add_directives:
    file: package://grocery/planar_test.dmd.yaml
"""

time_step=0.002
package_name = os.path.join(os.path.dirname(os.path.realpath(__file__)), "models/package.xml")

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
parser = Parser(plant)
parser.package_map().AddPackageXml(package_name)
AddPackagePaths(parser)
directives = LoadModelDirectivesFromString(model_directives)
ProcessModelDirectives(directives, parser)
plant.Finalize()

builder.ExportOutput(scene_graph.get_query_output_port(), "query_object")
diagram = builder.Build() #makemanipstation'un returnledigi sey
builder = DiagramBuilder()
station = builder.AddSystem(diagram)

visualizer = MeshcatVisualizer.AddToBuilder(builder, station.GetOutputPort("query_object"), meshcat)

diagram = builder.Build()
simulator = Simulator(diagram)
context = simulator.get_context()

simulator.AdvanceTo(0.1)
meshcat.Flush()  # Wait for the large object meshes to get to meshcat.

visualizer.StartRecording(True)
meshcat.AddButton("Stop Simulation", "Escape")
while simulator.get_context().get_time() < MAX_TIME and meshcat.GetButtonClicks("Stop Simulation") < 1:
    simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
visualizer.PublishRecording()

while True:
    pass