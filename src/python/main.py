import logging
import numpy as np
import os

from pydrake.all import (DiagramBuilder, MeshcatVisualizer, PortSwitch, Simulator, StartMeshcat, MultibodyPlant, Multiplexer, Demultiplexer)

from manipulation import running_as_notebook
from manipulation.scenarios import  ycb
from manipulation.meshcat_utils import MeshcatPoseSliders
                                    
import scenarios
from scenarios import JOINT_COUNT
import grasp_selector
import nodiffik_warnings
import planner as planner_class
import helpers

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
MAX_TIME = 160  # max duration after which the simulation is forced to end (recommended: ITEM_COUNT * 31)

def clutter_clearing_demo():
    meshcat.Delete()
    builder = DiagramBuilder()
    # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

    model_directives = """
directives:
- add_directives:
    file: package://grocery/clutter_w_cameras.dmd.yaml
"""
    i = 0
    # generate ITEM_COUNT items
    while i < ITEM_COUNT:
        object_num = rs.randint(len(ycb))
        if "cracker_box" in ycb[object_num] or "mustard" in ycb[object_num] or "sugar" in ycb[object_num]:
            # skip it. it's just too big!
            continue
        model_directives += f"""
- add_model:
    name: ycb{i}
#    file: package://drake/examples/manipulation_station/models/061_foam_brick.sdf
    file: package://grocery/meshes/{ycb[object_num]}
"""
        i += 1

    station = builder.AddSystem(
        scenarios.MakeManipulationStation(model_directives, time_step=0.001,
                                    package_xmls=[os.path.join(os.path.dirname(
                                       os.path.realpath(__file__)), "models/package.xml")]))
    plant = station.GetSubsystemByName("plant")

    # point cloud cropbox points
    cropPointA = [-.28, -.72, 0.36]
    cropPointB = [0.26, -.47, 0.57]

    x_bin_grasp_selector = builder.AddSystem(
        grasp_selector.GraspSelector(plant,
                      #plant.GetModelInstanceByName("shelves1"),
                      plant.GetFrameByName("shelves1_frame"),
                      camera_body_indices=[
                          plant.GetBodyIndices(
                              plant.GetModelInstanceByName("camera0"))[0],
                          plant.GetBodyIndices(
                              plant.GetModelInstanceByName("camera1"))[0],
                          plant.GetBodyIndices(
                              plant.GetModelInstanceByName("camera2"))[0]
                      ], cropPointA=cropPointA, cropPointB=cropPointB, meshcat=meshcat, running_as_notebook=running_as_notebook))
    builder.Connect(station.GetOutputPort("camera0_point_cloud"),x_bin_grasp_selector.get_input_port(0))
    builder.Connect(station.GetOutputPort("camera1_point_cloud"), x_bin_grasp_selector.get_input_port(1))
    builder.Connect(station.GetOutputPort("camera2_point_cloud"), x_bin_grasp_selector.get_input_port(2))
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

        iiwa_state_demux = builder.AddSystem(
            Demultiplexer([2, 7, 2, 7]))
        iiwa_state_mux = builder.AddSystem(
                    Multiplexer([7, 7]))
        builder.Connect(station.GetOutputPort("iiwa_state_estimated"),
                        iiwa_state_demux.get_input_port())
        builder.Connect(iiwa_state_demux.get_output_port(1),
                        iiwa_state_mux.get_input_port(0))
        builder.Connect(iiwa_state_demux.get_output_port(3),
                        iiwa_state_mux.get_input_port(1))
    
        builder.Connect(planner.GetOutputPort("X_WG"), diff_ik.get_input_port(0))
        #builder.Connect(iiwa_state_mux.get_output_port(), diff_ik.GetInputPort("robot_state"))
        builder.Connect(station.GetOutputPort("iiwa_state_estimated"), diff_ik.GetInputPort("robot_state"))
        builder.Connect(planner.GetOutputPort("reset_diff_ik"), diff_ik.GetInputPort("use_robot_state"))

        # mux = builder.AddSystem(Multiplexer([2, 7]))
        # builder.Connect(planner.GetOutputPort("base_position"),
        #                 mux.get_input_port(0))
        # builder.Connect(diff_ik.get_output_port(),
        #                 mux.get_input_port(1))
        # builder.Connect(mux.get_output_port(), switch.DeclareInputPort("diff_ik"))
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
    helpers.place_items(plant,plant_context, x=-0.20, y=-0.50, z=0.4)

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

