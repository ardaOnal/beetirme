import logging
import numpy as np
import os

from pydrake.all import (DiagramBuilder, MeshcatVisualizer, PortSwitch, Simulator, StartMeshcat, 
                         MultibodyPlant, Multiplexer, Demultiplexer, ConstantValueSource, AbstractValue)
import matplotlib.pyplot as plt
from matplotlib.pyplot import plot, draw, show, ion
import pydot
from IPython.display import HTML, SVG, display                         

from manipulation import running_as_notebook
from manipulation.scenarios import  ycb
from manipulation.meshcat_utils import MeshcatPoseSliders
                                    
import scenarios
from scenarios import JOINT_COUNT
import grasp_selector
import nodiffik_warnings
import planner as planner_class
import helpers
import models.env_generation as env

import pydot
from IPython.display import HTML, SVG, display

logging.getLogger("drake").addFilter(nodiffik_warnings.NoDiffIKWarnings())
logging.getLogger("drake").addFilter(nodiffik_warnings.NoSDFWarnings())

# Start the visualizer.
meshcat = StartMeshcat()

rs = np.random.RandomState()

SAVE_DIAGRAM_SVG = False
PREPICK_DISTANCE = 0.12
ITEM_COUNT = 3  # number of items to be generated
MAX_TIME = 10  # max duration after which the simulation is forced to end (recommended: ITEM_COUNT * 31)

def clutter_clearing_demo():
    meshcat.Delete()
    builder = DiagramBuilder()
    # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

    CONFIG = 1 # 0: row, 1: U shaped shelves

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
        ITEM_COUNT = num_shelves * items_per_shelf

    elif CONFIG == 1:
        side_shelf_count = 3
        no_of_sides = 3
        num_shelves = side_shelf_count * no_of_sides
        camera_per_shelf = 4
        camera_count = num_shelves * camera_per_shelf
        increment = -1
        shelf_start_point = 3
        items_per_shelf = 3
        ITEM_COUNT = num_shelves * items_per_shelf
        env.maze(start=shelf_start_point, side=side_shelf_count, no_of_sides=no_of_sides, increment=increment)


    model_directives = """
directives:
- add_directives:
    file: package://grocery/clutter_w_cameras.dmd.yaml
"""
    i = 0
    object_num = 0
    # generate ITEM_COUNT items
    while i < ITEM_COUNT:
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

    # point cloud cropbox points
    cropPointA = [-.28, -.72, 0.36]
    cropPointB = [0.26, -.47, 0.57]

    x_bin_grasp_selector = builder.AddSystem(
        grasp_selector.GraspSelector(plant,
                      #plant.GetModelInstanceByName("shelves1"),
                      plant.GetFrameByName("shelves1_origin"),
                      camera_body_indices=[
                          plant.GetBodyIndices(
                              plant.GetModelInstanceByName(f"camera{camera_no}_{shelf_no}"))[0]
                              for shelf_no in range(1, num_shelves+1) for camera_no in range(camera_per_shelf)
                      ], cropPointA=cropPointA, cropPointB=cropPointB, meshcat=meshcat, running_as_notebook=running_as_notebook, camera_count=camera_count,
                        diag=diag, station=station, camera_per_shelf=camera_per_shelf, num_shelves=num_shelves))
    
    cons = builder.AddSystem(ConstantValueSource(AbstractValue.Make(3)))
    builder.Connect(cons.get_output_port(0), x_bin_grasp_selector.GetInputPort("shelf_id"))

    for shelf_id in range(1, num_shelves+1):
        for i in range(4):
            builder.Connect(station.GetOutputPort(f"camera{i}_{shelf_id}_point_cloud"), x_bin_grasp_selector.GetInputPort(f"cloud{i}_s{shelf_id}"))
        builder.Connect(station.GetOutputPort(f"camera1_{shelf_id}_rgb_image"), x_bin_grasp_selector.GetInputPort(f"rgb_s{shelf_id}"))
        builder.Connect(station.GetOutputPort(f"camera1_{shelf_id}_depth_image"), x_bin_grasp_selector.GetInputPort(f"depth_s{shelf_id}"))
        
    builder.Connect(station.GetOutputPort("body_poses"), x_bin_grasp_selector.GetInputPort("body_poses"))

    # Add planner
    planner = builder.AddSystem(planner_class.Planner(plant, JOINT_COUNT, meshcat, rs, PREPICK_DISTANCE))
    builder.Connect(station.GetOutputPort("body_poses"), planner.GetInputPort("body_poses"))
    builder.Connect(x_bin_grasp_selector.get_output_port(), planner.GetInputPort("x_bin_grasp"))
    builder.Connect(station.GetOutputPort("wsg_state_measured"), planner.GetInputPort("wsg_state"))
    builder.Connect(station.GetOutputPort("iiwa_position_measured"), planner.GetInputPort("iiwa_position"))

    # The DiffIK and the direct position-control modes go through a PortSwitch
    switch = builder.AddSystem(PortSwitch(JOINT_COUNT))

    if False:
        # Set up mobile base differential inverse kinematics.
        robot = station.GetSubsystemByName("iiwa_controller").get_multibody_plant_for_control()
        diff_ik = scenarios.AddIiwaDifferentialIK(builder, robot)

        builder.Connect(planner.GetOutputPort("X_WG"), diff_ik.get_input_port(0))
        builder.Connect(station.GetOutputPort("iiwa_state_estimated"), diff_ik.GetInputPort("robot_state"))
        builder.Connect(planner.GetOutputPort("reset_diff_ik"), diff_ik.GetInputPort("use_robot_state"))

        builder.Connect(diff_ik.get_output_port(), switch.DeclareInputPort("diff_ik"))
    else:
        # Set up fixed base differential inverse kinematics.
        # create fixed plant
        fixed_plant = MultibodyPlant(time_step=0.001)
        controller_iiwa = scenarios.AddIiwa(fixed_plant, fixed=True)
        scenarios.AddWsg(fixed_plant, controller_iiwa, welded=True)
        fixed_plant.Finalize()

        diff_ik = scenarios.AddIiwaDifferentialIK(builder, fixed_plant)
    
        builder.Connect(planner.GetOutputPort("X_WG"), diff_ik.get_input_port(0))
        #builder.Connect(iiwa_state_mux.get_output_port(), diff_ik.GetInputPort("robot_state"))
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
        for shelf_index in range(num_shelves):
            if shelf_index < 6:
                x = 0
                y = 0.23
                z = -0.1
            else:
                x = 0.18
                y = -0.07
                z = -0.1
            X_SHELF = plant.GetFrameByName(f"shelves{shelf_index+1}_origin").CalcPoseInWorld(plant_context)
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

