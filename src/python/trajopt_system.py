from pydrake.all import (AbstractValue, InputPortIndex, LeafSystem,
                         PiecewisePolynomial, PiecewisePose, RigidTransform,
                         RollPitchYaw, Sphere, Rgba)
from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation.pick import (MakeGripperCommandTrajectory, MakeGripperPoseTrajectory)
from copy import copy
from kinematic import run_trajopt

import numpy as np

import planner_state
import pick

class KinematicTrajectoryOptimizer(LeafSystem):
    def __init__(self, plant, joint_count, meshcat, rs, prepick_distance):
        LeafSystem.__init__(self)
        # self.DeclareAbstractInputPort(
        #     "body_poses", AbstractValue.Make([RigidTransform()]))
        self._X_G_initial_index = self.DeclareAbstractInputPort(
            "X_G_initial", AbstractValue.Make(
                RigidTransform())).get_index()
        self._X_G_desired_index = self.DeclareAbstractInputPort(
            "X_G_desired", AbstractValue.Make(
                RigidTransform())).get_index()
        self._reset_trajopt_index = self.DeclareAbstractInputPort(
            "reset_trajopt", AbstractValue.Make(False)).get_index()
        #self._wsg_state_index = self.DeclareVectorInputPort("wsg_state", 2).get_index()
        
        # trajectory for joint positions
        self._traj_q_index = self.DeclareAbstractState(
            AbstractValue.Make(PiecewisePolynomial()))

        self.DeclareVectorOutputPort("joint_positions", joint_count,
            self.CalcIiwaPosition)

        #self.DeclareInitializationDiscreteUpdateEvent(self.Initialize)
        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0, self.Update)
        
        self.meshcat = meshcat
        self.rs = rs
        self.prepick_distance = prepick_distance

    def Update(self, context, state):
        reset_trajopt = self.get_input_port(
            self._reset_trajopt_index).Eval(context)      

        if reset_trajopt:
            print("res", reset_trajopt)
            self.RunTrajOpt(context, state)

    
    def RunTrajOpt(self, context, state):
        X_W_Start = self.get_input_port(
            self._X_G_initial_index).Eval(context)
    
        X_W_Goal = self.get_input_port(
            self._X_G_desired_index).Eval(context)
        
        q_traj = run_trajopt(X_W_Start, X_W_Goal)

        state.get_mutable_abstract_state(int(
            self._traj_q_index)).set_value(q_traj)


    def start_time(self, context):
        return context.get_abstract_state(
            int(self._traj_q_index)).get_value().start_time()

    def end_time(self, context):
        return context.get_abstract_state(
            int(self._traj_q_index)).get_value().end_time()

    def CalcIiwaPosition(self, context, output):
        traj_q = context.get_mutable_abstract_state(int(
                self._traj_q_index)).get_value()

        current_time = context.get_time()
        print(traj_q.end_time())
        if traj_q and traj_q.is_time_in_range(current_time):
            output.SetFromVector(traj_q.value(current_time))