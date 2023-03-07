import os
import numpy as np

from pydrake.all import (AddMultibodyPlantSceneGraph, DiagramBuilder, Parser, RigidTransform, RollPitchYaw)
from manipulation.scenarios import AddPackagePaths

# Another diagram for the objects the robot "knows about": gripper, cameras, bins.  Think of this as the model in the robot's head.
def make_internal_model():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)
    AddPackagePaths(parser)

    parser.package_map().AddPackageXml(os.path.join(os.path.dirname(os.path.realpath(__file__)), "models/package.xml"))

    
    parser.AddModels(
        "src/python/models/clutter_planning.dmd.yaml") # (Duzelttim) Bunun içindeki two_bins yaml'ini local yerine manipulationdan al diye setledim boku yemis olabilir
    plant.Finalize()
    return builder.Build()

def place_items(plant, plant_context, x, y, z):
    count = 0
    for body_index in plant.GetFloatingBaseBodies():
        tf = RigidTransform(
                RollPitchYaw(-np.pi/2, 0, 0),
                #[rs.uniform(-.20,0.23), rs.uniform(-.52, -.65), z])
                [x,y,z])
        plant.SetFreeBodyPose(plant_context,
                              plant.get_body(body_index),
                              tf)
        count += 1
        x += 0.2
        if count == 3:
            y -= 0.13
            x -= 0.5