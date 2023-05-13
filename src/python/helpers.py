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
    for body_index in range(start_index, end_index):
        coordinate_array = [shelf_frame.translation()[0]+x, shelf_frame.translation()[1]+y, shelf_frame.translation()[2]+z]
        if shelf_index < 6:
            roll = RollPitchYaw(-np.pi/2, 0, 0)
        else:
            roll = RollPitchYaw(-np.pi/2, 0, -np.pi/2)
        tf = RigidTransform(
                #RollPitchYaw(-np.pi/2, 0, 0),
                roll,
                #[rs.uniform(-.20,0.23), rs.uniform(-.52, -.65), z])
                coordinate_array)
        plant.SetFreeBodyPose(plant_context,
                              plant.get_body(list(plant.GetFloatingBaseBodies())[body_index]), # TO DO body index bozuyo
                              tf)
        count += 1
        if shelf_index < 6:
            y -= 0.2
        else:
            x -= 0.2
        if count == items_per_shelf:
            # y -= 0.13
            # x -= 0.5
            break