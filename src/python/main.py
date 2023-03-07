import logging
import numpy as np
import os

from pydrake.all import (DiagramBuilder, MeshcatVisualizer, PortSwitch, Simulator, StartMeshcat)

from manipulation import running_as_notebook
from manipulation.scenarios import  ycb
from manipulation.meshcat_utils import MeshcatPoseSliders
                                    
import scenarios
import grasp_selector
import nodiffik_warnings
import planner as planner_class
import helpers

logging.getLogger("drake").addFilter(nodiffik_warnings.NoDiffIKWarnings())

# Start the visualizer.
meshcat = StartMeshcat()

rs = np.random.RandomState()

JOINT_COUNT = 8
PREPICK_DISTANCE = 0.12
ITEM_COUNT = 5  # number of items to be generated
MAX_TIME = 160  # max duration after which the simulation is forced to end (recommended: ITEM_COUNT * 31)

def clutter_clearing_demo():
    meshcat.Delete()
    builder = DiagramBuilder()

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

    planner = builder.AddSystem(planner_class.Planner(plant, JOINT_COUNT, meshcat, rs, PREPICK_DISTANCE))
    builder.Connect(station.GetOutputPort("body_poses"), planner.GetInputPort("body_poses"))
    builder.Connect(x_bin_grasp_selector.get_output_port(), planner.GetInputPort("x_bin_grasp"))
    builder.Connect(station.GetOutputPort("wsg_state_measured"), planner.GetInputPort("wsg_state"))
    builder.Connect(station.GetOutputPort("iiwa_position_measured"), planner.GetInputPort("iiwa_position"))

    robot = station.GetSubsystemByName("iiwa_controller").get_multibody_plant_for_control()

    # Set up differential inverse kinematics.
    diff_ik = scenarios.AddIiwaDifferentialIK(builder, robot)
    builder.Connect(planner.GetOutputPort("X_WG"), diff_ik.get_input_port(0))
    builder.Connect(station.GetOutputPort("iiwa_state_estimated"), diff_ik.GetInputPort("robot_state"))
    builder.Connect(planner.GetOutputPort("reset_diff_ik"), diff_ik.GetInputPort("use_robot_state"))

    builder.Connect(planner.GetOutputPort("wsg_position"), station.GetInputPort("wsg_position"))

    # The DiffIK and the direct position-control modes go through a PortSwitch
    switch = builder.AddSystem(PortSwitch(JOINT_COUNT))
    builder.Connect(diff_ik.get_output_port(), switch.DeclareInputPort("diff_ik"))
    builder.Connect(planner.GetOutputPort("iiwa_position_command"), switch.DeclareInputPort("position"))
    builder.Connect(switch.get_output_port(), station.GetInputPort("iiwa_position"))
    builder.Connect(planner.GetOutputPort("control_mode"), switch.get_port_selector_input_port())

    visualizer = MeshcatVisualizer.AddToBuilder(builder, station.GetOutputPort("query_object"), meshcat)
    
    # Set up teleop widgets.
    teleop = builder.AddSystem(
        MeshcatPoseSliders(
            meshcat,
            min_range=MeshcatPoseSliders.MinRange(roll=0,
                                                  pitch=-0.5,
                                                  yaw=-np.pi,
                                                  x=-0.6,
                                                  y=-0.8,
                                                  z=0.0),
            max_range=MeshcatPoseSliders.MaxRange(roll=2 * np.pi,
                                                  pitch=np.pi,
                                                  yaw=np.pi,
                                                  x=0.8,
                                                  y=0.3,
                                                  z=1.1),
            body_index=plant.GetBodyByName("iiwa_link_7").index()))
    
    # builder.Connect(teleop.get_output_port(0), diff_ik.get_input_port(0))
    # builder.Connect(station.GetOutputPort("body_poses"), teleop.GetInputPort("body_poses"))
    # wsg_teleop = builder.AddSystem(WsgButton(meshcat))
    # builder.Connect(wsg_teleop.get_output_port(0),
    #                 station.GetInputPort("wsg_position"))
    
    diagram = builder.Build()

    simulator = Simulator(diagram)
    context = simulator.get_context()

    plant_context = plant.GetMyMutableContextFromRoot(context)
    helpers.place_items(plant,plant_context, x=-0.20, y=-0.50, z=0.4)

    simulator.AdvanceTo(0.1)
    meshcat.Flush()  # Wait for the large object meshes to get to meshcat.

    if True:
        visualizer.StartRecording(True)
        meshcat.AddButton("Stop Simulation", "Escape")
        while not planner._simulation_done and simulator.get_context().get_time() < MAX_TIME and meshcat.GetButtonClicks("Stop Simulation") < 1:
            simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
        visualizer.PublishRecording()

clutter_clearing_demo()

while True:
    pass

