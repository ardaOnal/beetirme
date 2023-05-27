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

def place_items(shelf_index, start_index, end_index, shelf_frame, plant, plant_context, x, y, z, items_per_shelf):
    count = 0
    bodies = sorted(list(plant.GetFloatingBaseBodies()))
    for body_index in range(start_index, end_index):
        body = plant.get_body(bodies[body_index])
        body_name = body.name()

        #coordinate_array = [shelf_frame.translation()[0]+x, shelf_frame.translation()[1]+y, shelf_frame.translation()[2]+z]
        
        pitch = -np.pi/2 if body_name == "base_link_sugar" or body_name == "base_link_cracker" else 0
        
        coordinate = [x, y, z]
        roll = RollPitchYaw(-np.pi/2, pitch, 0)

        # place sugar in different orientation to add variety
        if body_name == "base_link_sugar" and count == 1:
            roll = RollPitchYaw(-np.pi/2, 0, -np.pi/4)
            coordinate = [0.05, y, z]


        tf = RigidTransform(
                roll,
                coordinate)
        finalFrame = shelf_frame @ tf
        plant.SetFreeBodyPose(plant_context, body, finalFrame)
        count += 1
        y -= 0.2
        if count == items_per_shelf:
            break
