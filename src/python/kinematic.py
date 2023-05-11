import time

import os
import numpy as np
from pydrake.all import (AddMultibodyPlantSceneGraph, BsplineTrajectory,
                         DiagramBuilder, KinematicTrajectoryOptimization,
                         MeshcatVisualizer, MeshcatVisualizerParams,
                         MinimumDistanceConstraint, Parser, PositionConstraint,
                         Rgba, RigidTransform, Role, Solve, Sphere,
                         StartMeshcat, JointIndex, Simulator, RotationMatrix, PiecewisePolynomial, PidController, TrajectorySource,
                         InverseDynamicsController, Multiplexer, SchunkWsgPositionController, MakeMultibodyStateToWsgStateSystem, ModelInstanceIndex)

import pydrake.systems.framework
from pydrake.systems.primitives import ConstantVectorSource

from manipulation.meshcat_utils import PublishPositionTrajectory
from manipulation.scenarios import AddPackagePaths
from manipulation.scenarios import AddIiwa, AddShape, AddWsg, AddPackagePaths, MakeManipulationStation
from manipulation.utils import FindResource

#from scenarios import MakeManipulationStation, AddIiwa


def create_system_with_station():
    model_directives = """
    directives:
    - add_directives:
        file: package://grocery/two_bins_w_cameras.dmd.yaml
        #file: package://grocery/kinematic.dmd.yaml
    - add_directives:
        #file: package://grocery/iiwa_and_wsg_with_collision.dmd.yaml
        file: package://grocery/mobile_iiwa_with_collision.dmd.yaml    
    - add_model:
        name: foam_brick
        file: package://drake/examples/manipulation_station/models/061_foam_brick.sdf
        # file: package://grocery/meshes/005_tomato_soup_can.sdf 
    """
    diagram = MakeManipulationStation(model_directives, time_step=0.001,
                                    package_xmls=[os.path.join(os.path.dirname(
                                       os.path.realpath(__file__)), "models/package.xml")])
    return diagram


def make_internal_model():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    iiwa = AddIiwa(plant, collision_model="with_box_collision")
    wsg = AddWsg(plant, iiwa, welded=True, sphere=True)

    parser = Parser(plant)
    AddPackagePaths(parser)
    parser.package_map().AddPackageXml(os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "models/package.xml"))
    parser.AddModels(
        "src/python/models/two_bins_w_cameras.dmd.yaml")
    #parser.AddModels(
    #    "src/python/models/kinematic.dmd.yaml")
    plant.Finalize()
    return builder.Build()


def MakeGripperCommandTrajectory():
    """Constructs a WSG command trajectory from the plan "sketch"."""
    opened = np.array([0.107])
    closed = np.array([0.0])

    traj_wsg_command = PiecewisePolynomial.FirstOrderHold(
        [0.0, 0.4], np.hstack([[opened],[opened]]))
    traj_wsg_command.AppendFirstOrderSegment(0.7, closed)
    traj_wsg_command.AppendFirstOrderSegment(1.8, closed)
    traj_wsg_command.AppendFirstOrderSegment(2.1, opened)
    traj_wsg_command.AppendFirstOrderSegment(2.5, opened)
    return traj_wsg_command


def trajopt(plant, context, X_WStart, X_WGoal, q=None, duration=None):
    plant_context = plant.GetMyContextFromRoot(context)

    num_q = plant.num_positions()
    q0 = plant.GetPositions(plant_context)
    wsg = plant.GetModelInstanceByName("gripper")
    gripper_frame = plant.GetFrameByName("body", wsg)
    #iiwa = plant.GetModelInstanceByName("iiwa7")
    #mobile_base_y = plant.GetFrameByName("mobile_base_link", iiwa)

    trajopt = KinematicTrajectoryOptimization(plant.num_positions(), 10)
    prog = trajopt.get_mutable_prog()
    trajopt.AddDurationCost(1.0)
    trajopt.AddPathLengthCost(1.0)
    trajopt.AddPositionBounds(plant.GetPositionLowerLimits(),
                              plant.GetPositionUpperLimits())
    trajopt.AddVelocityBounds(plant.GetVelocityLowerLimits(),
                              plant.GetVelocityUpperLimits())

    if duration:
        trajopt.AddDurationConstraint(duration, duration)
    else:
        trajopt.AddDurationConstraint(.5, 50)

    # start constraint
    # base_start_constraint = PositionConstraint(plant, plant.world_frame(),
    #                                             [0, .2, 0],
    #                                             [0, .2, 0], mobile_base_y,
    #                                             [0, 0, 0], plant_context)

    start_constraint = PositionConstraint(plant, plant.world_frame(),
                                          X_WStart.translation(),
                                          X_WStart.translation(), gripper_frame,
                                          [0, 0, 0], plant_context)
    
    trajopt.AddPathPositionConstraint(start_constraint, 0)

    # for i in range(7):
    #     start_constraint = PositionConstraint(plant, plant.world_frame(),
    #                                         q[i],
    #                                         q[i], plant.GetFrameByName(f"iiwa_joint_{i+1}_parent", i+1),
    #                                         [0, 0, 0], plant_context)
        
    #     trajopt.AddPathPositionConstraint(start_constraint, 0)

    #trajopt.AddPathPositionConstraint(base_start_constraint, 0)

    if q:
        prog.AddBoundingBoxConstraint(q, q, trajopt.control_points()[:, 0])

    prog.AddQuadraticErrorCost(np.eye(num_q), q0,
                               trajopt.control_points()[:, 0])

    # goal constraint
    goal_constraint = PositionConstraint(plant, plant.world_frame(),
                                         X_WGoal.translation(),
                                         X_WGoal.translation(), gripper_frame,
                                         [0, 0, 0], plant_context)
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

    result = Solve(prog)
    if not result.is_success():
        print("Trajectory optimization failed")
        print(result.get_solver_id().name())

    return trajopt.ReconstructTrajectory(result)


def run_trajopt(X_WStart, X_WGoal, start_time=0, q=None, duration=None):
    # create an internal model
    internal_model = make_internal_model()
    internal_context = internal_model.CreateDefaultContext()
    internal_plant = internal_model.GetSubsystemByName("plant")

    # trajectory optimization
    trajectory = trajopt(internal_plant, internal_context, X_WStart, X_WGoal, q, duration)
    #plant.SetDefaultPositions(iiwa, trajectory.value(0))
    #internal_plant.SetDefaultPositions(trajectory.value(0))

    # create a discrete trajectory of joint positions and velocities
    times = np.append(np.arange(trajectory.start_time(), trajectory.end_time(), 0.001),
                      trajectory.end_time())
    #pos = [trajectory.value(trajectory.start_time()) for i in range(1000)]
    pos = [trajectory.value(t) for t in times]
    #pos += [trajectory.value(trajectory.end_time()) for i in range(1000)]
    position_stack = np.column_stack(pos)
    
    real_times = times + start_time
    p_traj = PiecewisePolynomial.FirstOrderHold(real_times, position_stack)
    return p_traj


def trajopt_shelves_demo():
    meshcat = StartMeshcat()
    meshcat.Delete()
    builder = DiagramBuilder()

    # set start & goal
    #X_WStart = RigidTransform([0, -0.8, 0.59])  # top
    #X_WStart = RigidTransform([0, -0.8, 0.35])  # middle
    #X_WStart = RigidTransform([0, -0.8, 0.07])  # bottom

    X_WStart = RigidTransform([0, -0.5, 0.4])  # middle

    meshcat.SetObject("start", Sphere(0.02), rgba=Rgba(.9, .1, .1, 1))
    meshcat.SetTransform("start", X_WStart)
    X_WGoal = RigidTransform([0.6, 0.1, 0.1])
    meshcat.SetObject("goal", Sphere(0.02), rgba=Rgba(.1, .9, .1, 1))
    meshcat.SetTransform("goal", X_WGoal)

    station = builder.AddSystem(create_system_with_station())
    plant = station.GetSubsystemByName("plant")
    iiwa = plant.GetModelInstanceByName("iiwa")
    wsg = plant.GetModelInstanceByName("wsg")
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link"),
                                X_WStart @ RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2)))

    p_traj = run_trajopt(X_WStart, X_WGoal)
    v_traj = p_traj.derivative()

    plant.SetDefaultPositions(iiwa, p_traj.value(0))
    plant.SetDefaultPositions(wsg, [-0.055, 0.055])

    p_source = builder.AddSystem(TrajectorySource(p_traj))
    builder.Connect(p_source.get_output_port(), station.GetInputPort("iiwa_position"))
    
    # Wsg controller.
    traj_wsg_command = MakeGripperCommandTrajectory()
    wsg_source = builder.AddSystem(TrajectorySource(traj_wsg_command))
    wsg_source.set_name("wsg_command")
    builder.Connect(wsg_source.get_output_port(),
                    station.GetInputPort("wsg_position"))

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat,
        MeshcatVisualizerParams(role=Role.kIllustration))

    # set up the simulator and run the simulation.
    diagram = builder.Build()
    simulator = Simulator(diagram)
    visualizer.StartRecording(False)
    simulator.AdvanceTo(p_traj.end_time())
    visualizer.PublishRecording()

if __name__ == "__main__":
    trajopt_shelves_demo()

    while True:
        pass
