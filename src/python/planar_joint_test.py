from pydrake.all import (DiagramBuilder, StartMeshcat, LoadModelDirectivesFromString, 
                         AddMultibodyPlantSceneGraph, Parser, ProcessModelDirectives, Simulator, 
                         MeshcatVisualizer, PlanarJoint, PidController)

from manipulation.meshcat_utils import MeshcatPoseSliders
from manipulation.utils import AddPackagePaths

import os
import numpy as np

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

# Add a revolute joint
joint = plant.GetJointByName("planar_joint")

# Define a PID controller
kp, ki, kd = 100.0, 10.0, 1.0
controller = PidController(kp=[100.0]*3, ki=[10.0]*3, kd=[1.0]*3)

# Connect the controller to the joint
builder.AddSystem(controller)
builder.Connect(plant.get_state_output_port(0), controller.get_input_port())
builder.Connect(controller.get_output_port(), joint.get_tau_output_port())

visualizer = MeshcatVisualizer.AddToBuilder(builder, station.GetOutputPort("query_object"), meshcat)

# # Set up teleop widgets.
# teleop = builder.AddSystem(
#     MeshcatPoseSliders(
#         meshcat,
#         min_range=MeshcatPoseSliders.MinRange(roll=0,
#                                                 pitch=-0.5,
#                                                 yaw=-np.pi,
#                                                 x=-0.6,
#                                                 y=-0.8,
#                                                 z=0.0),
#         max_range=MeshcatPoseSliders.MaxRange(roll=2 * np.pi,
#                                                 pitch=np.pi,
#                                                 yaw=np.pi,
#                                                 x=0.8,
#                                                 y=0.3,
#                                                 z=1.1),
#         body_index=plant.GetBodyByName("box_hand_1").index()))

# builder.Connect(teleop.get_output_port(0), station.get_input_port(0))
# builder.Connect(station.GetOutputPort("body_poses"), teleop.GetInputPort("body_poses"))
# wsg_teleop = builder.AddSystem(WsgButton(meshcat))
# builder.Connect(wsg_teleop.get_output_port(0),
#                 station.GetInputPort("wsg_position"))


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