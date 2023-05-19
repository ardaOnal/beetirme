import numpy as np

from pydrake.all import (AbstractValue, Concatenate, LeafSystem, PointCloud, RigidTransform, 
                         RollPitchYaw, Sphere, Rgba, Image, ImageRgba8U, ImageDepth16U, ImageDepth32F, PortDataType)

from manipulation.clutter import GenerateAntipodalGraspCandidate

import helpers

import segmentation
from  PIL import Image as PILImage
from lang_sam.utils import draw_image

import torchvision.transforms.functional as Tf
import matplotlib.pyplot as plt
from matplotlib.pyplot import plot, draw, show, ion

from config import DEBUG_MODE

SHELF_HEIGHT_LOWER_LIMIT = 0.36
SEGMENTATION_THRESHOLD = 0.4

# Takes 3 point clouds (in world coordinates) as input, and outputs and estimated pose for the items.
class GraspSelector(LeafSystem):
    def __init__(self, plant, shelf_instance, camera_count, camera_body_indices, meshcat, running_as_notebook, diag, station, camera_per_shelf, num_shelves):
        LeafSystem.__init__(self)
        model_point_cloud = AbstractValue.Make(PointCloud(0))
        cntxt31 = diag.CreateDefaultContext()
        # rgb_im = station.GetOutputPort('camera1_{}_rgb_image'.format(1)).Eval(cntxt31).data
        self.DeclareAbstractInputPort("item", AbstractValue.Make(("", 0)))

        for shelf_id in range(1, num_shelves+1):
            for i in range(4):
                self.DeclareAbstractInputPort(f"cloud{i}_s{shelf_id}", model_point_cloud)
            self.DeclareAbstractInputPort(f"rgb_s{shelf_id}", AbstractValue.Make(Image(31,31)))
            self.DeclareAbstractInputPort(f"depth_s{shelf_id}", AbstractValue.Make(ImageDepth32F(31,31)))

        self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()]))

        port = self.DeclareAbstractOutputPort(
            "grasp_selection", lambda: AbstractValue.Make(
                (np.inf, RigidTransform())), self.SelectGrasp)

        port.disable_caching_by_default()

        self.station = station
        self.diag = diag
        self.cntxt31 = diag.CreateDefaultContext()

        self._internal_model = helpers.make_internal_model()
        self._internal_model_context = self._internal_model.CreateDefaultContext()
        self._rng = np.random.default_rng()
        self._camera_body_indices = camera_body_indices
        self.running_as_notebook = running_as_notebook
        self.camera_count = camera_count

        self.lang_sam_model = segmentation.get_lang_sam("vit_b")

        self.cam_info = []
        self.X_WC_Cam1 = []
        for shelf_id in range(1, num_shelves+1): 
            cam1 = diag.GetSubsystemByName(f"camera0_{shelf_id}")
            self.cam_info.append(cam1.depth_camera_info())

            cam1_context = cam1.GetMyMutableContextFromRoot(cntxt31)
            self.X_WC_Cam1.append(cam1.body_pose_in_world_output_port().Eval(cam1_context))

        self.meshcat = meshcat

    def project_depth_to_pC(self, depth_pixel, shelf_id, uv=None):
        """
        project depth pixels to points in camera frame
        using pinhole camera model
        Input:
            depth_pixels: numpy array of (nx3) or (3,)
        Output:
            pC: 3D point in camera frame, numpy array of (nx3)
        """
        # switch u,v due to python convention
        v = depth_pixel[:, 0]
        u = depth_pixel[:, 1]
        Z = depth_pixel[:, 2]
        # read camera intrinsics
        cx = self.cam_info[shelf_id-1].center_x()
        cy = self.cam_info[shelf_id-1].center_y()
        fx = self.cam_info[shelf_id-1].focal_x()
        fy = self.cam_info[shelf_id-1].focal_y()
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        pC = np.c_[X, Y, Z]
        return pC

    # MAKE DYNAMIC
    # def SelectGrasp(self, context, output):
    #     output.set_value((np.inf, RigidTransform()))

    def SelectGrasp(self, context, output):
        shelf_id = self.GetInputPort("item").Eval(context)[1]
        # print("shelf", shelf_id)

        rgb_im = self.GetInputPort(f"rgb_s{shelf_id}").Eval(context).data # TO DO make dynamic
        image_pil = PILImage.fromarray(rgb_im).convert("RGB")
        # plt.imshow(image_pil)
        # plt.show()

        text_prompt = self.GetInputPort("item").Eval(context)[0]
        #print("picking", text_prompt)
        #text_prompt = 'sugar_box'
        #text_prompt = 'dark_blue_canned_spaghetti'
        if DEBUG_MODE: print("---Segmenting objects---")
        masks, boxes, phrases, logits = self.lang_sam_model.predict(image_pil, text_prompt)
        # print("masks", masks)
        # print("masks shape", masks.shape)
        # print("boxes", boxes)
        # print("phrases", phrases)
        # print("logits", logits)

        selected_index = 0
        for i in range(len(logits)):
            if logits[i] > SEGMENTATION_THRESHOLD:
                selected_index = i

        if DEBUG_MODE: print("---Segmentation complete---")
        # compute bottom left and top right corner of the segmented pixel
        smallest_sum = [2**30,0,0] # value and coordinates
        largest_sum = [0,0,0]

        if len(masks) > 0:
            for x in range(masks[selected_index].shape[0]):
                for y in range(masks[selected_index].shape[1]):
                    if masks[selected_index][x][y] == True and x + y > largest_sum[0]:
                        largest_sum = [x+y, x, y]
                    if masks[selected_index][x][y] == True and x + y < smallest_sum[0]:
                        smallest_sum = [x+y, x, y]

            np_image = np.array(image_pil)
            result = draw_image(np_image, masks, boxes, phrases)
            # plt.imshow(result)
            # plt.show()

            depth_im = self.GetInputPort(f"depth_s{shelf_id}").Eval(context).data.squeeze() # todo

            img_h, img_w = depth_im.shape
            v_range = np.arange(img_h)
            u_range = np.arange(img_w)
            depth_u, depth_v = np.meshgrid(u_range, v_range)
            depth_pnts = np.dstack([depth_v, depth_u, depth_im])
            depth_pnts = depth_pnts.reshape([img_h * img_w, 3])

            # point poses in camera frame
            pC = self.project_depth_to_pC(depth_pnts, shelf_id)
            pC = np.reshape(pC,(480,640,3))

            #print("CROP POINTS", pC[smallest_sum[1]][smallest_sum[2]], pC[largest_sum[1]][largest_sum[2]])

            X_Crop1 = self.X_WC_Cam1[shelf_id-1] @ RigidTransform(pC[smallest_sum[1]][smallest_sum[2]])
            #print("CROPPED FRAME1", X_Crop1)
            X_Crop2 = self.X_WC_Cam1[shelf_id-1] @ RigidTransform(pC[largest_sum[1]][largest_sum[2]])
            #print("CROPPED FRAME2", X_Crop2)

            # Solves: Failure at perception/point_cloud.cc:350 in Crop(): condition '(lower_xyz.array() <= upper_xyz.array()).all()' failed.
            X_Crop1_Array = [X_Crop1.translation()[0], X_Crop1.translation()[1], X_Crop1.translation()[2]]
            X_Crop2_Array = [X_Crop2.translation()[0], X_Crop2.translation()[1], X_Crop2.translation()[2]]
            if X_Crop1_Array[0] > X_Crop2_Array[0]:
                tmp = X_Crop2_Array[0]
                X_Crop2_Array[0] = X_Crop1_Array[0]
                X_Crop1_Array[0] = tmp
            if X_Crop1_Array[1] > X_Crop2_Array[1]:
                tmp = X_Crop2_Array[1]
                X_Crop2_Array[1] = X_Crop1_Array[1]
                X_Crop1_Array[1] = tmp
            if X_Crop1_Array[2] > X_Crop2_Array[2]:
                tmp = X_Crop2_Array[2]
                X_Crop2_Array[2] = X_Crop1_Array[2]
                X_Crop1_Array[2] = tmp

            # widen the area of the crop
            X_Crop1_Array[0] = X_Crop1_Array[0] - 0.035
            X_Crop1_Array[1] = X_Crop1_Array[1] - 0.035
            X_Crop1_Array[2] = X_Crop1_Array[2] - 0.035

            X_Crop2_Array[0] = X_Crop2_Array[0] + 0.035
            X_Crop2_Array[1] = X_Crop2_Array[1] + 0.035
            X_Crop2_Array[2] = X_Crop2_Array[2] + 0.035

            if X_Crop1_Array[2] < SHELF_HEIGHT_LOWER_LIMIT:
                X_Crop1_Array[2] = SHELF_HEIGHT_LOWER_LIMIT

            #print(X_Crop1_Array)
            #print(X_Crop2_Array)

            
            # put spheres to corners of the crop box
            X_B = RigidTransform([0,0,0])
            a = X_B.multiply(X_Crop1_Array)
            b = X_B.multiply(X_Crop2_Array)
            if DEBUG_MODE:
                self.meshcat.SetObject("pick1", Sphere(0.01), rgba=Rgba(.9, .1, .1, 1))
                self.meshcat.SetTransform("pick1", RigidTransform(a))
                self.meshcat.SetObject("pick2", Sphere(0.01), rgba=Rgba(.1, .9, .1, 1))
                self.meshcat.SetTransform("pick2", RigidTransform(b))
                print("---Calculating grasp---")

            body_poses = self.GetInputPort("body_poses").Eval(context) # TO DO
            pcd = []
            for i in range(4): # TO
                cloud = self.GetInputPort(f"cloud{i}_s{shelf_id}").Eval(context) # TO DO
                pcd.append(cloud.Crop(np.array(X_Crop1_Array), np.array(X_Crop2_Array)))
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
            for i in range(100):
                cost, X_G = GenerateAntipodalGraspCandidate(
                    self._internal_model, self._internal_model_context,
                    down_sampled_pcd, self._rng)
                if np.isfinite(cost):
                    costs.append(cost)
                    X_Gs.append(X_G)

            if len(costs) == 0:
                # Didn't find a viable grasp candidate
                print("Didn't find a viable grasp candidate")
                X_WG = RigidTransform(RollPitchYaw(-np.pi / 2, 0, np.pi / 2),
                                    [0.5, 0, 0.22])
                output.set_value((np.inf, X_WG))
            else:
                best = np.argmin(costs)
                output.set_value((costs[best], X_Gs[best]))
        else:
            print("Could not segment any of the objects")


