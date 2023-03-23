import numpy as np
import os
import matplotlib.pyplot as plt

from pydrake.common import FindResourceOrThrow, temp_directory
from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    StartMeshcat,
)

from manipulation.utils import AddPackagePaths, FindResource
from manipulation.meshcat_utils import PublishPositionTrajectory
from IPython.display import clear_output
from manipulation.meshcat_utils import MeshcatPoseSliders
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.visualization import ModelVisualizer
from manipulation import running_as_notebook
from manipulation.scenarios import MakeManipulationStation, AddIiwa, AddWsg, AddPlanarIiwa
from pydrake.all import ( AddMultibodyPlantSceneGraph, AngleAxis, BsplineTrajectory, 
                         DiagramBuilder, FindResourceOrThrow, Integrator,InverseKinematics,
                         JacobianWrtVariable, KinematicTrajectoryOptimization, LeafSystem, MeshcatVisualizer,
                         MinimumDistanceConstraint, MultibodyPlant, MultibodyPositionToGeometryPose,
                         Parser, PiecewisePolynomial, PiecewisePose, PositionConstraint, 
                         Quaternion, Rgba, RigidTransform, RotationMatrix,
                         SceneGraph, Simulator, Solve,  StartMeshcat, TrajectorySource, InverseDynamicsController, SpatialInertia, UnitInertia, LoadModelDirectivesFromString,
                         ProcessModelDirectives)
from manipulation.utils import colorize_labels

from pydrake.all import (AddMultibodyPlantSceneGraph, Box,
                         ConnectPlanarSceneGraphVisualizer, CoulombFriction,
                         DiagramBuilder, DrakeVisualizer, FindResourceOrThrow,
                         FixedOffsetFrame, JointIndex, Parser, PlanarJoint,
                         RandomGenerator, RigidTransform, RollPitchYaw,
                         RotationMatrix, Simulator,
                         UniformlyRandomRotationMatrix, PlanarJoint, Context,JointActuator, PrismaticJoint, RevoluteJoint, WeldJoint)

from manipulation import running_as_notebook
from manipulation.scenarios import (AddRgbdSensor, AddShape, ycb)

import json
from IPython.display import HTML, SVG, display
import pydot

meshcat = StartMeshcat()

meshcat.Flush()
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.05)

parser = Parser(plant)

package_name = os.path.join(os.path.dirname(os.path.realpath(__file__)), "models/package.xml")

model_directives = """
directives:
- add_directives:
    file: package://grocery/clutter_w_cameras.dmd.yaml
"""

parser = Parser(plant)
parser.package_map().AddPackageXml(package_name)
AddPackagePaths(parser)
directives = LoadModelDirectivesFromString(model_directives)
ProcessModelDirectives(directives, parser)


plant.Finalize()

visualizer = MeshcatVisualizer.AddToBuilder(
    builder, scene_graph, meshcat)

plant_context = plant.CreateDefaultContext()

kp=[1] * plant.num_positions()
ki=[1] * plant.num_positions()
kd=[1] * plant.num_positions()

planar_controller = builder.AddSystem(
    InverseDynamicsController(plant, kp, ki, kd, False))
planar_controller.set_name("planar_controller")
builder.Connect(plant.get_state_output_port(),
                planar_controller.get_input_port_estimated_state())
builder.Connect(planar_controller.get_output_port_control(),
                plant.get_actuation_input_port())

diagram = builder.Build()
context = diagram.CreateDefaultContext()

states = [0.2, 0.4, 0.3, 0.5, 1, 0.1, 0.5, 0.1, 1.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0]
planar_controller.GetInputPort('desired_state').FixValue(
    planar_controller.GetMyMutableContextFromRoot(context), states)

simulator = Simulator(diagram, context)
"""simulator.get_mutable_integrator().set_fixed_step_mode(True)"""
visualizer.StartRecording()
"""SVG(pydot.graph_from_dot_data(diagram.GetGraphvizString())[0]).create_svg()"""

print(context)
simulator.AdvanceTo(12.0)
visualizer.StopRecording()
visualizer.PublishRecording()

while True:
    continue

