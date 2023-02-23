import time

import os
import numpy as np
from pydrake.all import (AddMultibodyPlantSceneGraph, BsplineTrajectory,
                         DiagramBuilder, KinematicTrajectoryOptimization,
                         MeshcatVisualizer, MeshcatVisualizerParams,
                         MinimumDistanceConstraint, Parser, PositionConstraint,
                         Rgba, RigidTransform, Role, Solve, Sphere,
                         StartMeshcat, JointIndex, Simulator)

import pydrake.systems.framework

#from manipulation.meshcat_utils import PublishPositionTrajectory
from manipulation.scenarios import AddIiwa, AddShape, AddWsg, AddPackagePaths, MakeManipulationStation
from manipulation.utils import  FindResource

# Start the visualizer.
meshcat = StartMeshcat()    

def make_internal_model():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    iiwa = AddIiwa(plant, collision_model="with_box_collision")
    wsg = AddWsg(plant, iiwa, welded=True, sphere=True)

    parser = Parser(plant)
    AddPackagePaths(parser)
    bin = parser.AddModelFromFile(
        FindResource("models/shelves.sdf"))
    plant.WeldFrames(plant.world_frame(),
                    plant.GetFrameByName("shelves_body", bin),
                    RigidTransform([0.88, 0, 0.4]))

    X_WStart = RigidTransform([0.8, 0, 0.65])
    meshcat.SetObject("start", Sphere(0.02), rgba=Rgba(.9, .1, .1, 1))
    meshcat.SetTransform("start", X_WStart)
    X_WGoal = RigidTransform([0, -0.4, 0])
    meshcat.SetObject("goal", Sphere(0.02), rgba=Rgba(.1, .9, .1, 1))
    meshcat.SetTransform("goal", X_WGoal)

    #parser.package_map().AddPackageXml(os.path.join(os.path.dirname(os.path.realpath(__file__)), "models/package.xml"))
    
    plant.Finalize()
    return builder.Build()

def trajopt_shelves_demo():
    meshcat.Delete()
    builder = DiagramBuilder()


    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    iiwa = AddIiwa(plant, collision_model="with_box_collision")
    wsg = AddWsg(plant, iiwa, welded=True, sphere=True)

    X_WStart = RigidTransform([0.8, 0, 0.65])
    meshcat.SetObject("start", Sphere(0.02), rgba=Rgba(.9, .1, .1, 1))
    meshcat.SetTransform("start", X_WStart)
    X_WGoal = RigidTransform([0, -0.4, 0])
    meshcat.SetObject("goal", Sphere(0.02), rgba=Rgba(.1, .9, .1, 1))
    meshcat.SetTransform("goal", X_WGoal)
    
    parser = Parser(plant)
    bin = parser.AddModelFromFile(
        FindResource("models/shelves.sdf"))
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("shelves_body", bin),
                     RigidTransform([0.88, 0, 0.4]))

    parser.package_map().AddPackageXml(os.path.join(os.path.dirname(os.path.realpath(__file__)), "models/package.xml"))
    parser.AddModelFromFile(
        "src/python/models/meshes/004_sugar_box.sdf")
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link_sugar"), RigidTransform([0.88, 0, 1.4]))
    # add free body #
    #parser.AddModelFromFile(FindResource("models/061_foam_brick_w_visual_contact_spheres.sdf"))
    #plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link"), RigidTransform([0.88, 0, 1.4]))

    plant.Finalize()

    internal_model = make_internal_model()
    internal_context = internal_model.CreateDefaultContext()
    internal_plant = internal_model.GetSubsystemByName("plant")
    internal_scene_graph = internal_model.GetSubsystemByName("scene_graph")

#     model_directives = """
# directives:
# - add_directives:
#     file: package://grocery/iiwa_and_wsg_with_collision.dmd.yaml
# - add_model:
#     name: shelves    
#     file: package://manipulation/shelves.sdf
# - add_weld:
#     parent: world
#     child: shelves_body
#     X_PC:
#         translation: [0.88, 0, 0.4]
#         rotation: !Rpy { deg: [0, 0, 0]}
# """

#     station = builder.AddSystem(
#         MakeManipulationStation(model_directives, time_step=0.001,
#                                     package_xmls=[os.path.join(os.path.dirname(
#                                        os.path.realpath(__file__)), "models/package.xml")]))
#     plant = station.GetSubsystemByName("plant")
#     print(plant.num_positions(), plant.GetPositionLowerLimits())
#     for ind in range(plant.num_joints()):
#         joint = plant.get_joint(JointIndex(ind))
#         print(type(joint))
#     scene_graph = station.GetOutputPort("query_object")
#     #print([sys.get_name() for sys in station.GetSystems()])
#     wsg = plant.GetModelInstanceByName("wsg")



    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(role=Role.kIllustration))
    collision_visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(prefix="collision", role=Role.kProximity))
    meshcat.SetProperty("collision", "visible", False)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    internal_plant_context = plant.GetMyContextFromRoot(context)
    trajectory = trajopt(internal_plant, internal_context, X_WStart, X_WGoal)


    PublishPositionTrajectory(trajectory, context, plant, visualizer)
    collision_visualizer.ForcedPublish(
        collision_visualizer.GetMyContextFromRoot(context))
    

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
        print(positions)
        plant.SetPositions(plant_context, np.append(trajectory.value(t), positions))
        visualizer.ForcedPublish(visualizer_context)

    visualizer.StopRecording()
    visualizer.PublishRecording()

trajopt_shelves_demo()


while True: pass
