import logging
import numpy as np
import os

from pydrake.all import (DiagramBuilder, MeshcatVisualizer, PortSwitch, Simulator, StartMeshcat, 
                         MultibodyPlant, Multiplexer, Demultiplexer, ConstantValueSource, AbstractValue,
                         RigidTransform, RollPitchYaw)
import matplotlib.pyplot as plt
from matplotlib.pyplot import plot, draw, show, ion
import pydot
from IPython.display import HTML, SVG, display                         
from manipulation.meshcat_utils import AddMeshcatTriad

from manipulation import running_as_notebook
from manipulation.scenarios import  ycb
from manipulation.meshcat_utils import MeshcatPoseSliders
                                    
import scenarios
from scenarios import JOINT_COUNT
from grasp_selector import GraspSelector
import nodiffik_warnings
import planner as planner_class
import helpers
import models.env_generation as env
from parse_list import select_items

import pydot
from IPython.display import HTML, SVG, display

logging.getLogger("drake").addFilter(nodiffik_warnings.NoDiffIKWarnings())
logging.getLogger("drake").addFilter(nodiffik_warnings.NoSDFWarnings())

# Start the visualizer.
meshcat = StartMeshcat()

rs = np.random.RandomState()

from config import *

def clutter_clearing_demo():
    meshcat.Delete()
    builder = DiagramBuilder()

    if CONFIG == 0:
        row_count = 3
        shelf_row_count = 2
        num_shelves = row_count * shelf_row_count
        camera_per_shelf = 4
        camera_count = num_shelves * camera_per_shelf
        row_increment = 3.3
        shelf_increment = 1.1
        row_start_point=-.6
        shelf_start_point = 0
        env.grid(row_start_point=row_start_point, row_count=row_count, shelf_row_count=shelf_row_count, 
                shelf_start_point=shelf_start_point, row_increment=row_increment, shelf_increment=shelf_increment)

        items_per_shelf = 3
        item_count = num_shelves * items_per_shelf

    elif CONFIG == 1:
        side_shelf_count = 3
        no_of_sides = 2
        num_shelves = side_shelf_count * no_of_sides
        camera_per_shelf = 4
        camera_count = num_shelves * camera_per_shelf
        increment = -1
        shelf_start_point = 3
        items_per_shelf = 3
        item_count = num_shelves * items_per_shelf
        env.maze(start=shelf_start_point, side=side_shelf_count, no_of_sides=no_of_sides, increment=increment)


    model_directives = """
directives:
- add_directives:
    file: package://grocery/clutter_w_cameras.dmd.yaml
"""
    i = 0
    object_num = 0
    # generate item_count items
    while i < item_count:
        # if "cracker_box" in ycb[object_num] or "mustard" in ycb[object_num] or "sugar" in ycb[object_num]:
        #     # skip it. it's just too big!
        #     continue
        model_directives += f"""
- add_model:
    name: ycb{i}
    file: package://grocery/meshes/{ycb[object_num]}
"""
        i += 1
        if (i % items_per_shelf == 0):
            object_num = (object_num + 1) % len(ycb)
    diag = scenarios.MakeManipulationStation(model_directives, time_step=0.001,
                                    package_xmls=[os.path.join(os.path.dirname(
                                       os.path.realpath(__file__)), "models/package.xml")])
    station = builder.AddSystem(diag)
    plant = station.GetSubsystemByName("plant")

    grasp_selector = builder.AddSystem(
        GraspSelector(plant,
                      plant.GetFrameByName("shelves1_origin"),
                      camera_body_indices=[
                          plant.GetBodyIndices(
                              plant.GetModelInstanceByName(f"camera{camera_no}_{shelf_no}"))[0]
                              for shelf_no in range(1, num_shelves+1) for camera_no in range(camera_per_shelf)
                      ], meshcat=meshcat, running_as_notebook=running_as_notebook, camera_count=camera_count,
                        diag=diag, station=station, camera_per_shelf=camera_per_shelf, num_shelves=num_shelves))

    for shelf_id in range(1, num_shelves+1):
        for i in range(4):
            builder.Connect(station.GetOutputPort(f"camera{i}_{shelf_id}_point_cloud"), grasp_selector.GetInputPort(f"cloud{i}_s{shelf_id}"))
        builder.Connect(station.GetOutputPort(f"camera0_{shelf_id}_rgb_image"), grasp_selector.GetInputPort(f"rgb_s{shelf_id}"))
        builder.Connect(station.GetOutputPort(f"camera0_{shelf_id}_depth_image"), grasp_selector.GetInputPort(f"depth_s{shelf_id}"))
        
    builder.Connect(station.GetOutputPort("body_poses"), grasp_selector.GetInputPort("body_poses"))

    # Determine the pose of the base of the robot when visiting each shelf
    delivery_position = [-2, 0, 0] if CONFIG == 0 else [0, 0, 0]
    shelf_poses = {0: RigidTransform(delivery_position)}
    con = plant.CreateDefaultContext()
    X_Shelf_Robot = RigidTransform(RollPitchYaw(0, 0, -np.pi/2), [0.6, 0, -.6085])
    for shelf_id in range(1, num_shelves+1):
        X_shelf = plant.GetFrameByName(f"shelves{shelf_id}_origin").CalcPoseInWorld(con) @ X_Shelf_Robot
        shelf_poses[shelf_id] = X_shelf
        if DEBUG_MODE: AddMeshcatTriad(meshcat, f"shelf{shelf_id}", X_PT=X_shelf)

    # select items from GUI
    shopping_list = select_items(plant, items_per_shelf)
    item_list = []
    for item in shopping_list:
        for i in range(item[1]):
            item_list.append((item[0], item[2]))
    print(item_list)

    # Add planner
    planner = builder.AddSystem(planner_class.Planner(plant, JOINT_COUNT, meshcat, rs, PREPICK_DISTANCE, shelf_poses, item_list))
    builder.Connect(station.GetOutputPort("body_poses"), planner.GetInputPort("body_poses"))
    builder.Connect(grasp_selector.get_output_port(), planner.GetInputPort("x_bin_grasp"))
    builder.Connect(station.GetOutputPort("wsg_state_measured"), planner.GetInputPort("wsg_state"))
    builder.Connect(station.GetOutputPort("iiwa_position_measured"), planner.GetInputPort("iiwa_position"))

    # Provide shelf id input to grasp selector and planner
    #cons = builder.AddSystem(ConstantValueSource(AbstractValue.Make(2)))
    builder.Connect(planner.GetOutputPort("item"), grasp_selector.GetInputPort("item"))

    # The DiffIK and the direct position-control modes go through a PortSwitch
    switch = builder.AddSystem(PortSwitch(JOINT_COUNT))

    # Set up fixed base differential inverse kinematics.
    # # create fixed plant
    # fixed_plant = MultibodyPlant(time_step=0.001)
    # controller_iiwa = scenarios.AddIiwa(fixed_plant, fixed=True)
    # scenarios.AddWsg(fixed_plant, controller_iiwa, welded=True)
    # fixed_plant.Finalize()

    robot = station.GetSubsystemByName(
        "iiwa_controller").get_multibody_plant_for_control()
    diff_ik = scenarios.AddIiwaDifferentialIK(builder, robot)

    builder.Connect(planner.GetOutputPort("X_WG"), diff_ik.get_input_port(0))
    builder.Connect(station.GetOutputPort("iiwa_state_estimated"), diff_ik.GetInputPort("robot_state"))
    builder.Connect(planner.GetOutputPort("reset_diff_ik"), diff_ik.GetInputPort("use_robot_state"))

    builder.Connect(diff_ik.get_output_port(), switch.DeclareInputPort("diff_ik"))
        

    builder.Connect(planner.GetOutputPort("wsg_position"), station.GetInputPort("wsg_position"))


    builder.Connect(planner.GetOutputPort("iiwa_position_command"), switch.DeclareInputPort("position"))
    builder.Connect(switch.get_output_port(), station.GetInputPort("iiwa_position"))
    builder.Connect(planner.GetOutputPort("control_mode"), switch.get_port_selector_input_port())

    visualizer = MeshcatVisualizer.AddToBuilder(builder, station.GetOutputPort("query_object"), meshcat)    
    diagram = builder.Build()

    if SAVE_DIAGRAM_SVG:
        svg = SVG(pydot.graph_from_dot_data(diagram.GetGraphvizString())[0].create_svg())
        with open('diagram.svg', 'w') as f:
            f.write(svg.data)

    simulator = Simulator(diagram)
    context = simulator.get_context()    
  
    plant_context = plant.GetMyMutableContextFromRoot(context)

    # x = -0.20 
    # y = -0.6
    # z = 0.4

    if CONFIG == 0: # row config
        x = shelf_start_point - 0.2
        y = row_start_point
        z = 0.4
        slice_counter = 0
        for i in range(row_count):
            for j in range(num_shelves):
                items = list(plant.GetFloatingBaseBodies())[slice_counter:slice_counter+items_per_shelf]
                helpers.place_items(items, plant,plant_context, x=x, y=y, z=z, items_per_shelf=items_per_shelf)
                x += shelf_increment
                slice_counter = slice_counter + items_per_shelf
            y = y + row_increment
            x = shelf_start_point - 0.2
    elif CONFIG == 1:
        x = shelf_start_point - 0.2
        y = shelf_start_point
        z = 0.4
        # x = 0
        # y = 0.2
        # z = 3.5

        slice_cnt = 0
        for shelf_index in range(1, num_shelves+1):
            if shelf_index <= 6:
                x = -0.06
                y = 0.23
                z = -0.1
            else:
                x = 0.18
                y = -0.07
                z = -0.1
            X_SHELF = plant.GetFrameByName(f"shelves{shelf_index}_origin").CalcPoseInWorld(plant_context)
            helpers.place_items(shelf_index, slice_cnt, slice_cnt+items_per_shelf, X_SHELF, plant, plant_context, x=x, y=y, z=z, items_per_shelf=items_per_shelf)
            slice_cnt = slice_cnt + items_per_shelf
        

    # run simulation
    simulator.AdvanceTo(0.1)
    meshcat.Flush()  # Wait for the large object meshes to get to meshcat.
    visualizer.StartRecording(True)
    meshcat.AddButton("Stop Simulation", "Escape")
    while not planner._simulation_done and simulator.get_context().get_time() < MAX_TIME and meshcat.GetButtonClicks("Stop Simulation") < 1:
        #print(switch.get_output_port(0).Eval(switch.GetMyMutableContextFromRoot(context)))
        simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
    visualizer.PublishRecording()

clutter_clearing_demo()

while True: pass

