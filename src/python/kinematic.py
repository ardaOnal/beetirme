import time

import os
import numpy as np
from pydrake.all import (AddMultibodyPlantSceneGraph, BsplineTrajectory,
                         DiagramBuilder, KinematicTrajectoryOptimization,
                         MeshcatVisualizer, MeshcatVisualizerParams,
                         MinimumDistanceConstraint, Parser, PositionConstraint,
                         Rgba, RigidTransform, Role, Solve, Sphere,
                         StartMeshcat, JointIndex, Simulator, RotationMatrix, PiecewisePolynomial)

import pydrake.systems.framework
from pydrake.systems.primitives import ConstantVectorSource

from manipulation.meshcat_utils import PublishPositionTrajectory
from manipulation.scenarios import AddIiwa, AddShape, AddWsg, AddPackagePaths, MakeManipulationStation
from manipulation.utils import  FindResource

# Start the visualizer.
meshcat = StartMeshcat()    

def create_system():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    iiwa = AddIiwa(plant, collision_model="with_box_collision")
    wsg = AddWsg(plant, iiwa, welded=True, sphere=True)

    parser = Parser(plant)
    AddPackagePaths(parser)
    parser.package_map().AddPackageXml(os.path.join(os.path.dirname(os.path.realpath(__file__)), "models/package.xml"))
    parser.AddModels(
        "src/python/models/kinematic.dmd.yaml")
    
    return builder, parser, plant, scene_graph, iiwa, wsg
    

def make_internal_model():
    builder, parser, plant, scene_graph, iiwa, wsg = create_system()
    plant.Finalize()
    return builder.Build()

def PublishPositionTrajectory(trajectory,
                              root_context,
                              plant,
                              visualizer,
                              time_step=1.0 / 33.0):
    plant_context = plant.GetMyContextFromRoot(root_context)
    visualizer_context = visualizer.GetMyContextFromRoot(root_context)

    visualizer.StartRecording(False)

    for t in np.append(
            np.arange(trajectory.start_time(), trajectory.end_time(),
                      time_step), trajectory.end_time()):
        root_context.SetTime(t)
        positions = np.array(plant.GetPositions(plant_context)[7:])
        #print(positions)
        plant.SetPositions(plant_context, np.append(trajectory.value(t), positions))
        visualizer.ForcedPublish(visualizer_context)

    visualizer.StopRecording()
    visualizer.PublishRecording()

def run_simulation(simulator, visualizer):
    #simulator.AdvanceTo(0.1)
    #meshcat.Flush()
    visualizer.StartRecording(False)
    meshcat.AddButton("Stop Simulation", "Escape")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
    visualizer.PublishRecording()

def run_trajectory(simulator,
                trajectory,
                root_context,
                plant,
                visualizer,
                time_step=0.001):
    plant_context = plant.GetMyContextFromRoot(root_context)
    visualizer_context = visualizer.GetMyContextFromRoot(root_context)
    
    meshcat.Flush()
    visualizer.StartRecording(False)

    times = np.append(np.arange(trajectory.start_time(), trajectory.end_time(), time_step), 
                                trajectory.end_time())
    visualizer.StartRecording(False)
    meshcat.AddButton("Stop Simulation", "Escape")
    for t in times:
        simulator.AdvanceTo(t)
        root_context.SetTime(t)
        positions = np.array(plant.GetPositions(plant_context)[7:])
        plant.SetPositions(plant_context, np.append(trajectory.value(t), positions))
        visualizer.ForcedPublish(visualizer_context)
    visualizer.StopRecording()
    visualizer.PublishRecording()

def trajopt(plant, context, X_WStart, X_WGoal):
    plant_context = plant.GetMyContextFromRoot(context)
    
    num_q = plant.num_positions()
    q0 = plant.GetPositions(plant_context)
    wsg = plant.GetModelInstanceByName("gripper")
    gripper_frame = plant.GetFrameByName("body", wsg)

    trajopt = KinematicTrajectoryOptimization(plant.num_positions(), 10)
    prog = trajopt.get_mutable_prog()
    trajopt.AddDurationCost(1.0)
    trajopt.AddPathLengthCost(1.0)
    trajopt.AddPositionBounds(plant.GetPositionLowerLimits(),
                              plant.GetPositionUpperLimits())
    trajopt.AddVelocityBounds(plant.GetVelocityLowerLimits(),
                              plant.GetVelocityUpperLimits())

    trajopt.AddDurationConstraint(.5, 50)

    # start constraint
    start_constraint = PositionConstraint(plant, plant.world_frame(),
                                          X_WStart.translation(),
                                          X_WStart.translation(), gripper_frame,
                                          [0, 0.1, 0], plant_context)
    trajopt.AddPathPositionConstraint(start_constraint, 0)
    prog.AddQuadraticErrorCost(np.eye(num_q), q0,
                               trajopt.control_points()[:, 0])

    # goal constraint
    goal_constraint = PositionConstraint(plant, plant.world_frame(),
                                         X_WGoal.translation(),
                                         X_WGoal.translation(), gripper_frame,
                                         [0, 0.1, 0], plant_context)
    trajopt.AddPathPositionConstraint(goal_constraint, 1)
    prog.AddQuadraticErrorCost(np.eye(num_q), q0,
                               trajopt.control_points()[:, -1])

    # start and end with zero velocity
    trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros(
        (num_q, 1)), 0)
    trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros(
        (num_q, 1)), 1)

    # Solve once without the collisions and set that as the initial guess for
    # the version with collisions.
    result = Solve(prog)
    if not result.is_success():
        print("Trajectory optimization failed, even without collisions!")
        print(result.get_solver_id().name())
    trajopt.SetInitialGuess(trajopt.ReconstructTrajectory(result))

    # collision constraints
    collision_constraint = MinimumDistanceConstraint(plant, 0.001,
                                                     plant_context, None, 0.01)
    evaluate_at_s = np.linspace(0, 1, 50)
    for s in evaluate_at_s:
        trajopt.AddPathPositionConstraint(collision_constraint, s)

    def PlotPath(control_points):
        traj = BsplineTrajectory(trajopt.basis(),
                                 control_points.reshape((3, -1)))
        meshcat.SetLine('positions_path',
                         traj.vector_values(np.linspace(0, 1, 50)))

    #prog.AddVisualizationCallback(PlotPath,
    #                              trajopt.control_points().reshape((-1,)))
    result = Solve(prog)
    if not result.is_success():
        print("Trajectory optimization failed")
        print(result.get_solver_id().name())
    
    return trajopt.ReconstructTrajectory(result)

def trajopt_shelves_demo():
    meshcat.Delete()

    # set start & goal
    #X_WStart = RigidTransform([0, -0.8, 0.59])  # top
    #X_WStart = RigidTransform([0, -0.8, 0.33])  # middle
    X_WStart = RigidTransform([0, -0.8, 0.07])  # bottom
    meshcat.SetObject("start", Sphere(0.02), rgba=Rgba(.9, .1, .1, 1))
    meshcat.SetTransform("start", X_WStart)
    X_WGoal = RigidTransform([0.6, 0.1, 0.1])
    meshcat.SetObject("goal", Sphere(0.02), rgba=Rgba(.1, .9, .1, 1))
    meshcat.SetTransform("goal", X_WGoal)

    # create actual system
    builder, parser, plant, scene_graph, iiwa, wsg = create_system()
    parser.AddModelFromFile("src/python/models/meshes/005_tomato_soup_can.sdf")
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link_soup"), 
                                 X_WStart @ RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2)))
    plant.Finalize()

    # create an internal model
    internal_model = make_internal_model()
    internal_context = internal_model.CreateDefaultContext()
    internal_plant = internal_model.GetSubsystemByName("plant")
    internal_scene_graph = internal_model.GetSubsystemByName("scene_graph")

    # trajectory optimization
    trajectory = trajopt(internal_plant, internal_context, X_WStart, X_WGoal)

    # build diagram and simulation
    nu = plant.num_actuated_dofs(iiwa)
    u0 = np.zeros(nu)
    constant = builder.AddSystem(ConstantVectorSource(u0))
    builder.Connect(
        constant.get_output_port(0),
        plant.get_actuation_input_port(iiwa))
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(role=Role.kIllustration))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()

    # run simulation
    #PublishPositionTrajectory(trajectory, context, plant, visualizer)
    run_trajectory(simulator, trajectory, context, plant, visualizer)
    #run_simulation(simulator, visualizer)

#trajopt_shelves_demo()

def trajopt_shelves_demo2():
    meshcat.Delete()

    # set start & goal
    #X_WStart = RigidTransform([0, -0.8, 0.59])  # top
    #X_WStart = RigidTransform([0, -0.8, 0.33])  # middle
    X_WStart = RigidTransform([0, -0.8, 0.07])  # bottom
    meshcat.SetObject("start", Sphere(0.02), rgba=Rgba(.9, .1, .1, 1))
    meshcat.SetTransform("start", X_WStart)
    X_WGoal = RigidTransform([0.6, 0.1, 0.1])
    meshcat.SetObject("goal", Sphere(0.02), rgba=Rgba(.1, .9, .1, 1))
    meshcat.SetTransform("goal", X_WGoal)

    # create actual system
    builder = DiagramBuilder()

    model_directives = """
    directives:
    - add_directives:
        file: package://grocery/iiwa_and_wsg_with_collision.dmd.yaml
    - add_directives:
        file: package://grocery/kinematic.dmd.yaml
    """
    station = builder.AddSystem(
        MakeManipulationStation(model_directives, time_step=0.001,
                                    package_xmls=[os.path.join(os.path.dirname(
                                       os.path.realpath(__file__)), "models/package.xml")]))
    plant = station.GetSubsystemByName("plant")

    #parser.AddModelFromFile("src/python/models/meshes/005_tomato_soup_can.sdf")
    #plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link_soup"), 
    #                             X_WStart @ RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2)))

    # create an internal model
    internal_model = make_internal_model()
    internal_context = internal_model.CreateDefaultContext()
    internal_plant = internal_model.GetSubsystemByName("plant")
    internal_scene_graph = internal_model.GetSubsystemByName("scene_graph")

    # trajectory optimization
    traj = trajopt(internal_plant, internal_context, X_WStart, X_WGoal)


    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()

    # run simulation
    traj_pp = PiecewisePolynomial.CubicFromBezier(traj)


trajopt_shelves_demo2()

while True: pass
