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
    #print("shelf index: ", shelf_index, " item: ", plant.get_body(sorted(list(plant.GetFloatingBaseBodies()))[start_index]))
    bodies = sorted(list(plant.GetFloatingBaseBodies()))
    for body_index in range(start_index, end_index):
        body = plant.get_body(bodies[body_index])
        body_name = body.name()

        coordinate_array = [shelf_frame.translation()[0]+x, shelf_frame.translation()[1]+y, shelf_frame.translation()[2]+z]
        
        pitch = -np.pi/2 if body_name == "base_link_sugar" or body_name == "base_link_cracker" else 0

        if shelf_index <= 6:
            roll = RollPitchYaw(-np.pi/2, pitch, 0)
        else:
            roll = RollPitchYaw(-np.pi/2, pitch, -np.pi/2)
        tf = RigidTransform(
                #RollPitchYaw(-np.pi/2, 0, 0),
                roll,
                #[rs.uniform(-.20,0.23), rs.uniform(-.52, -.65), z])
                coordinate_array)
        plant.SetFreeBodyPose(plant_context,
                              body, # TO DO body index bozuyo
                              tf)
        count += 1
        if shelf_index <= 6:
            y -= 0.2
        else:
            x -= 0.2
        if count == items_per_shelf:
            # y -= 0.13
            # x -= 0.5
            break


def place_items_row_config(shelf_index, start_index, end_index, shelf_frame, plant, plant_context, x, y, z, items_per_shelf):
    count = 0
    #print("shelf index: ", shelf_index, " item: ", plant.get_body(sorted(list(plant.GetFloatingBaseBodies()))[start_index]))
    bodies = sorted(list(plant.GetFloatingBaseBodies()))
    for body_index in range(start_index, end_index):
        body = plant.get_body(bodies[body_index])
        body_name = body.name()

        coordinate_array = [shelf_frame.translation()[0]+x, shelf_frame.translation()[1]+y, shelf_frame.translation()[2]+z]
        
        pitch = -np.pi/2 if body_name == "base_link_sugar" or body_name == "base_link_cracker" else 0

        # if shelf_index <= 6:
        #     roll = RollPitchYaw(-np.pi/2, pitch, 0)
        # else:
        roll = RollPitchYaw(-np.pi/2, pitch, -np.pi/2)
        tf = RigidTransform(
                #RollPitchYaw(-np.pi/2, 0, 0),
                roll,
                #[rs.uniform(-.20,0.23), rs.uniform(-.52, -.65), z])
                coordinate_array)
        plant.SetFreeBodyPose(plant_context,
                              body, # TO DO body index bozuyo
                              tf)
        count += 1
        # if shelf_index <= 6:
        #     y -= 0.2
        # else:
        x += 0.2
        if count == items_per_shelf:
            # y -= 0.13
            # x -= 0.5
            break