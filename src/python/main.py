import logging
from copy import copy
from enum import Enum

import numpy as np
import os
from pydrake.all import (AbstractValue, AddMultibodyPlantSceneGraph, AngleAxis,
                         Concatenate, DiagramBuilder, InputPortIndex,
                         LeafSystem, MeshcatVisualizer, Parser,
                         PiecewisePolynomial, PiecewisePose, PointCloud,
                         PortSwitch, RandomGenerator, RigidTransform,
                         RollPitchYaw, Simulator, StartMeshcat,
                         UniformlyRandomRotationMatrix, Sphere, Rgba, RotationMatrix)

from manipulation import FindResource, running_as_notebook
from manipulation.clutter import GenerateAntipodalGraspCandidate
from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation.pick import (MakeGripperCommandTrajectory, #MakeGripperFrames,
                               MakeGripperPoseTrajectory)
from manipulation.scenarios import (AddIiwaDifferentialIK, AddPackagePaths, ycb)
                                    #MakeManipulationStation
                                    
import scenarios
import pick


class NoDiffIKWarnings(logging.Filter):
    def filter(self, record):
        return not record.getMessage().startswith('Differential IK')

logging.getLogger("drake").addFilter(NoDiffIKWarnings())

# Start the visualizer.
meshcat = StartMeshcat()

rs = np.random.RandomState()

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

# Takes 3 point clouds (in world coordinates) as input, and outputs and estimated pose for the items.
class GraspSelector(LeafSystem):
    def __init__(self, plant, shelf_instance, camera_body_indices, cropPointA, cropPointB):
        LeafSystem.__init__(self)
        model_point_cloud = AbstractValue.Make(PointCloud(0))
        self.DeclareAbstractInputPort("cloud0_W", model_point_cloud)
        self.DeclareAbstractInputPort("cloud1_W", model_point_cloud)
        self.DeclareAbstractInputPort("cloud2_W", model_point_cloud)
        self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()]))

        port = self.DeclareAbstractOutputPort(
            "grasp_selection", lambda: AbstractValue.Make(
                (np.inf, RigidTransform())), self.SelectGrasp)
        port.disable_caching_by_default()

        # Compute crop box.
        context = plant.CreateDefaultContext()
        X_B = RigidTransform([0,0,0])
        #a = X_B.multiply([-.28, -.72, 0.36])
        #b = X_B.multiply([0.26, -.47, 0.57])
        a = X_B.multiply(cropPointA)
        b = X_B.multiply(cropPointB)
        
        if True: # corners of the crop box
            meshcat.SetObject("pick1", Sphere(0.01), rgba=Rgba(.9, .1, .1, 1))
            meshcat.SetTransform("pick1", RigidTransform(a))
            meshcat.SetObject("pick2", Sphere(0.01), rgba=Rgba(.1, .9, .1, 1))
            meshcat.SetTransform("pick2", RigidTransform(b))

        self._crop_lower = np.minimum(a,b)
        self._crop_upper = np.maximum(a,b)

        self._internal_model = make_internal_model()
        self._internal_model_context = self._internal_model.CreateDefaultContext()
        self._rng = np.random.default_rng()
        self._camera_body_indices = camera_body_indices

    def SelectGrasp(self, context, output):
        body_poses = self.get_input_port(3).Eval(context)
        pcd = []
        for i in range(3):
            cloud = self.get_input_port(i).Eval(context)
            pcd.append(cloud.Crop(self._crop_lower, self._crop_upper))
            pcd[i].EstimateNormals(radius=0.1, num_closest=30)

            # Flip normals toward camera
            X_WC = body_poses[self._camera_body_indices[i]]
            pcd[i].FlipNormalsTowardPoint(X_WC.translation())
        merged_pcd = Concatenate(pcd)
        down_sampled_pcd = merged_pcd.VoxelizedDownSample(voxel_size=0.005)

        costs = []
        X_Gs = []
        # TODO(russt): Take the randomness from an input port, and re-enable
        # caching.
        for i in range(100 if running_as_notebook else 2):
            cost, X_G = GenerateAntipodalGraspCandidate(
                self._internal_model, self._internal_model_context,
                down_sampled_pcd, self._rng)
            if np.isfinite(cost):
                costs.append(cost)
                X_Gs.append(X_G)

        if len(costs) == 0:
            # Didn't find a viable grasp candidate
            X_WG = RigidTransform(RollPitchYaw(-np.pi / 2, 0, np.pi / 2),
                                  [0.5, 0, 0.22])
            output.set_value((np.inf, X_WG))
        else:
            best = np.argmin(costs)
            output.set_value((costs[best], X_Gs[best]))

class PlannerState(Enum):
    WAIT_FOR_OBJECTS_TO_SETTLE = 1
    PICKING_FROM_SHELF_1 = 2
    GO_HOME = 3


PREPICK_DISTANCE = 0.12

class Planner(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self._gripper_body_index = plant.GetBodyByName("body").index()
        self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()]))
        self._x_bin_grasp_index = self.DeclareAbstractInputPort(
            "x_bin_grasp", AbstractValue.Make(
                (np.inf, RigidTransform()))).get_index()
        self._wsg_state_index = self.DeclareVectorInputPort("wsg_state", 2).get_index()

        self._mode_index = self.DeclareAbstractState(
            AbstractValue.Make(PlannerState.WAIT_FOR_OBJECTS_TO_SETTLE))
        self._traj_X_G_index = self.DeclareAbstractState(
            AbstractValue.Make(PiecewisePose()))
        self._traj_wsg_index = self.DeclareAbstractState(
            AbstractValue.Make(PiecewisePolynomial()))
        self._times_index = self.DeclareAbstractState(AbstractValue.Make(
            {"initial": 0.0}))
        self._attempts_index = self.DeclareDiscreteState(1)

        self.DeclareAbstractOutputPort(
            "X_WG", lambda: AbstractValue.Make(RigidTransform()),
            self.CalcGripperPose)
        self.DeclareVectorOutputPort("wsg_position", 1, self.CalcWsgPosition)

        # For GoHome mode.
        num_positions = 7
        self._iiwa_position_index = self.DeclareVectorInputPort(
            "iiwa_position", num_positions).get_index()
        self.DeclareAbstractOutputPort(
            "control_mode", lambda: AbstractValue.Make(InputPortIndex(0)),
            self.CalcControlMode)
        self.DeclareAbstractOutputPort(
            "reset_diff_ik", lambda: AbstractValue.Make(False),
            self.CalcDiffIKReset)
        self._q0_index = self.DeclareDiscreteState(num_positions)  # for q0
        self._traj_q_index = self.DeclareAbstractState(
            AbstractValue.Make(PiecewisePolynomial()))
        self.DeclareVectorOutputPort("iiwa_position_command", num_positions,
                                     self.CalcIiwaPosition)
        self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)

        self._simulation_done = False
        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0.0, self.Update)

    def Update(self, context, state):
        if self._simulation_done:
            return

        mode = context.get_abstract_state(int(self._mode_index)).get_value()

        current_time = context.get_time()
        times = context.get_abstract_state(int(
            self._times_index)).get_value()

        if mode == PlannerState.WAIT_FOR_OBJECTS_TO_SETTLE:
            if context.get_time() - times["initial"] > 1.0:
                self.Plan(context, state)
            return
        elif mode == PlannerState.GO_HOME:
            traj_q = context.get_mutable_abstract_state(int(
                self._traj_q_index)).get_value()
            if not traj_q.is_time_in_range(current_time):
                self.Plan(context, state)
            return

        # If we are between pick and place and the gripper is closed, then
        # we've missed or dropped the object.  Time to replan.
        if (current_time > times["postpick"] and
                current_time < times["preplace"]):
            wsg_state = self.get_input_port(self._wsg_state_index).Eval(context)
            if wsg_state[0] < 0.01:
                attempts = state.get_mutable_discrete_state(
                    int(self._attempts_index)).get_mutable_value()
                
                # exit
                if attempts[0] > 5:
                    attempts[0] = 0
                    return
                
                attempts[0] += 1
                state.get_mutable_abstract_state(int(
                    self._mode_index)).set_value(
                        PlannerState.WAIT_FOR_OBJECTS_TO_SETTLE)
                times = {"initial": current_time}
                state.get_mutable_abstract_state(int(
                    self._times_index)).set_value(times)
                X_G = self.get_input_port(0).Eval(context)[int(
                    self._gripper_body_index)]
                state.get_mutable_abstract_state(int(
                    self._traj_X_G_index)).set_value(PiecewisePose.MakeLinear([current_time, np.inf], [X_G, X_G]))
                return

        traj_X_G = context.get_abstract_state(int(
            self._traj_X_G_index)).get_value()
        if not traj_X_G.is_time_in_range(current_time):
            self.Plan(context, state)
            return

        X_G = self.get_input_port(0).Eval(context)[int(self._gripper_body_index)]

        # stop and replan
        if np.linalg.norm(traj_X_G.GetPose(current_time).translation()
                - X_G.translation()) > 0.2:
            self.GoHome(context, state)
            return

    def GoHome(self, context, state):
        print("Replanning due to large tracking error.")
        state.get_mutable_abstract_state(int(
            self._mode_index)).set_value(
                PlannerState.GO_HOME)
        q = self.get_input_port(self._iiwa_position_index).Eval(context)
        q0 = copy(context.get_discrete_state(self._q0_index).get_value())
        q0[0] = q[0]  # Safer to not reset the first joint.

        current_time = context.get_time()
        q_traj = PiecewisePolynomial.FirstOrderHold(
            [current_time, current_time + 5.0], np.vstack((q, q0)).T)
        state.get_mutable_abstract_state(int(
            self._traj_q_index)).set_value(q_traj)


    def Plan(self, context, state):
        mode = copy(
            state.get_mutable_abstract_state(int(self._mode_index)).get_value())

        X_G = {
            "initial":
                self.get_input_port(0).Eval(context)
                [int(self._gripper_body_index)]
        }

        # pick pose calculation
        cost = np.inf
        mode = PlannerState.PICKING_FROM_SHELF_1
        for i in range(5):
            cost, X_G["pick"] = self.get_input_port(
                self._x_bin_grasp_index).Eval(context)
            
            #X_G["pick"] = X_G["pick"] @ RigidTransform(RotationMatrix.MakeXRotation(-np.pi / 2))
            
            if not np.isinf(cost):
                break
        
        if np.isinf(cost):
            self._simulation_done = True
            print("Could not find a valid grasp in either bin after 5 attempts")
            return
        state.get_mutable_abstract_state(int(self._mode_index)).set_value(mode)

        # place pose calculation
        if mode == PlannerState.PICKING_FROM_SHELF_1:
            x_range = [.35, .65]
            y_range = [0, .35]

            # Place in Y bin:
            X_G["place"] = RigidTransform(
                RollPitchYaw(-np.pi / 2, 0, np.pi / 2),
                [rs.uniform(x_range[0], x_range[1]),
                 rs.uniform(y_range[0], y_range[1]),
                 0])
                
            meshcat.SetObject("place1", Sphere(0.02), rgba=Rgba(.9, .1, .1, 1))
            meshcat.SetTransform("place1", RigidTransform([x_range[0], y_range[0], 0]))
            meshcat.SetObject("place2", Sphere(0.02), rgba=Rgba(.1, .9, .1, 1))
            meshcat.SetTransform("place2", RigidTransform([x_range[1], y_range[1], 0]))

        # plan trajectory
        X_G, times = pick.MakeGripperFrames(X_G, t0=context.get_time(), prepick_distance=PREPICK_DISTANCE)
        print(
            f"t={int(context.get_time())}s - Planned {int(times['postplace'] - times['initial'])} seconds trajectory for picking from the shelf."
        )
        state.get_mutable_abstract_state(int(
            self._times_index)).set_value(times)


        if True:  # Useful for debugging
            #AddMeshcatTriad(meshcat, "X_Oinitial", X_PT=X_O["initial"])
            #AddMeshcatTriad(meshcat, "X_Gpickwrotat", X_PT=X_G["pick"] @ RigidTransform(RotationMatrix.MakeXRotation(np.pi / 10)))
            AddMeshcatTriad(meshcat, "X_Gprepick", X_PT=X_G["prepick"])
            AddMeshcatTriad(meshcat, "X_Gpick", X_PT=X_G["pick"])
            AddMeshcatTriad(meshcat, "X_Gplace", X_PT=X_G["place"])

        traj_X_G = MakeGripperPoseTrajectory(X_G, times)
        traj_wsg_command = MakeGripperCommandTrajectory(times)

        state.get_mutable_abstract_state(int(
            self._traj_X_G_index)).set_value(traj_X_G)
        state.get_mutable_abstract_state(int(
            self._traj_wsg_index)).set_value(traj_wsg_command)

    def start_time(self, context):
        return context.get_abstract_state(
            int(self._traj_X_G_index)).get_value().start_time()

    def end_time(self, context):
        return context.get_abstract_state(
            int(self._traj_X_G_index)).get_value().end_time()

    def CalcGripperPose(self, context, output):
        mode = context.get_abstract_state(int(self._mode_index)).get_value()

        traj_X_G = context.get_abstract_state(int(
            self._traj_X_G_index)).get_value()
        if (traj_X_G.get_number_of_segments() > 0 and
                traj_X_G.is_time_in_range(context.get_time())):
            # Evaluate the trajectory at the current time, and write it to the
            # output port.
            output.set_value(
                context.get_abstract_state(int(
                    self._traj_X_G_index)).get_value().GetPose(
                        context.get_time()))
            return

        # Command the current position (note: this is not particularly good if the velocity is non-zero)
        output.set_value(self.get_input_port(0).Eval(context)
            [int(self._gripper_body_index)])

    def CalcWsgPosition(self, context, output):
        mode = context.get_abstract_state(int(self._mode_index)).get_value()
        opened = np.array([0.107])
        closed = np.array([0.0])

        if mode == PlannerState.GO_HOME:
            # Command the open position
            output.SetFromVector([opened])
            return

        traj_wsg = context.get_abstract_state(int(
            self._traj_wsg_index)).get_value()
        if (traj_wsg.get_number_of_segments() > 0 and
                traj_wsg.is_time_in_range(context.get_time())):
            # Evaluate the trajectory at the current time, and write it to the
            # output port.
            output.SetFromVector(traj_wsg.value(context.get_time()))
            return

        # Command the open position
        output.SetFromVector([opened])

    def CalcControlMode(self, context, output):
        mode = context.get_abstract_state(int(self._mode_index)).get_value()

        if mode == PlannerState.GO_HOME:
            output.set_value(InputPortIndex(2))  # Go Home
        else:
            output.set_value(InputPortIndex(1))  # Diff IK

    def CalcDiffIKReset(self, context, output):
        mode = context.get_abstract_state(int(self._mode_index)).get_value()

        if mode == PlannerState.GO_HOME:
            output.set_value(True)
        else:
            output.set_value(False)

    def Initialize(self, context, discrete_state):
        discrete_state.set_value(
            int(self._q0_index),
            self.get_input_port(int(self._iiwa_position_index)).Eval(context))

    def CalcIiwaPosition(self, context, output):
        traj_q = context.get_mutable_abstract_state(int(
                self._traj_q_index)).get_value()

        output.SetFromVector(traj_q.value(context.get_time()))


def place_items(plant, plant_context, x, y, z):
    count = 0
    for body_index in plant.GetFloatingBaseBodies():
        tf = RigidTransform(
                RollPitchYaw(-np.pi/2, 0, 0),
                #[rs.uniform(-.20,0.23), rs.uniform(-.52, -.65), z])
                [x,y,z])
        plant.SetFreeBodyPose(plant_context,
                              plant.get_body(body_index),
                              tf)
        count += 1
        x += 0.2
        if count == 3:
            y -= 0.13
            x -= 0.5

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
        GraspSelector(plant,
                      #plant.GetModelInstanceByName("shelves1"),
                      plant.GetFrameByName("shelves1_frame"),
                      camera_body_indices=[
                          plant.GetBodyIndices(
                              plant.GetModelInstanceByName("camera0"))[0],
                          plant.GetBodyIndices(
                              plant.GetModelInstanceByName("camera1"))[0],
                          plant.GetBodyIndices(
                              plant.GetModelInstanceByName("camera2"))[0]
                      ], cropPointA=cropPointA, cropPointB=cropPointB))
    builder.Connect(station.GetOutputPort("camera0_point_cloud"),x_bin_grasp_selector.get_input_port(0))
    builder.Connect(station.GetOutputPort("camera1_point_cloud"),
                    x_bin_grasp_selector.get_input_port(1))
    builder.Connect(station.GetOutputPort("camera2_point_cloud"),
                    x_bin_grasp_selector.get_input_port(2))
    builder.Connect(station.GetOutputPort("body_poses"),
                    x_bin_grasp_selector.GetInputPort("body_poses"))

    planner = builder.AddSystem(Planner(plant))
    builder.Connect(station.GetOutputPort("body_poses"),
                    planner.GetInputPort("body_poses"))
    builder.Connect(x_bin_grasp_selector.get_output_port(),
                    planner.GetInputPort("x_bin_grasp"))
    builder.Connect(station.GetOutputPort("wsg_state_measured"),
                    planner.GetInputPort("wsg_state"))
    builder.Connect(station.GetOutputPort("iiwa_position_measured"),
                    planner.GetInputPort("iiwa_position"))

    robot = station.GetSubsystemByName(
        "iiwa_controller").get_multibody_plant_for_control()

    # Set up differential inverse kinematics.
    diff_ik = AddIiwaDifferentialIK(builder, robot)
    builder.Connect(planner.GetOutputPort("X_WG"),
                    diff_ik.get_input_port(0))
    builder.Connect(station.GetOutputPort("iiwa_state_estimated"),
                    diff_ik.GetInputPort("robot_state"))
    builder.Connect(planner.GetOutputPort("reset_diff_ik"),
                    diff_ik.GetInputPort("use_robot_state"))

    builder.Connect(planner.GetOutputPort("wsg_position"),
                    station.GetInputPort("wsg_position"))

    # The DiffIK and the direct position-control modes go through a PortSwitch
    switch = builder.AddSystem(PortSwitch(7))
    builder.Connect(diff_ik.get_output_port(),
                    switch.DeclareInputPort("diff_ik"))
    builder.Connect(planner.GetOutputPort("iiwa_position_command"),
                    switch.DeclareInputPort("position"))
    builder.Connect(switch.get_output_port(),
                    station.GetInputPort("iiwa_position"))
    builder.Connect(planner.GetOutputPort("control_mode"),
                    switch.get_port_selector_input_port())

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat)
    diagram = builder.Build()

    simulator = Simulator(diagram)
    context = simulator.get_context()

    plant_context = plant.GetMyMutableContextFromRoot(context)
    place_items(plant,plant_context, x=-0.20, y=-0.50, z=0.4)

    simulator.AdvanceTo(0.1)
    meshcat.Flush()  # Wait for the large object meshes to get to meshcat.

    # if running_as_notebook:
    if True:
        visualizer.StartRecording(False)
        meshcat.AddButton("Stop Simulation", "Escape")
        while not planner._simulation_done and simulator.get_context().get_time() < MAX_TIME and meshcat.GetButtonClicks("Stop Simulation") < 1:
            simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
        visualizer.PublishRecording()

clutter_clearing_demo()

while True:
    pass

