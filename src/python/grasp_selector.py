import numpy as np

from pydrake.all import (AbstractValue, Concatenate, LeafSystem, PointCloud, RigidTransform, 
                         RollPitchYaw, Sphere, Rgba)

from manipulation.clutter import GenerateAntipodalGraspCandidate

import helpers

# Takes 3 point clouds (in world coordinates) as input, and outputs and estimated pose for the items.
class GraspSelector(LeafSystem):
    def __init__(self, plant, shelf_instance, camera_body_indices, cropPointA, cropPointB, meshcat, running_as_notebook):
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

        self._internal_model = helpers.make_internal_model()
        self._internal_model_context = self._internal_model.CreateDefaultContext()
        self._rng = np.random.default_rng()
        self._camera_body_indices = camera_body_indices
        self.running_as_notebook = running_as_notebook

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
        for i in range(100 if self.running_as_notebook else 2):
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