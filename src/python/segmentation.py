# https://deepnote.com/workspace/Manipulation-ac8201a1-470a-4c77-afd0-2cc45bc229ff/project/092-Segmentation-Antipodal-Grasping-7d8360cd-0aca-4d6e-b704-a3a7d9b77615/notebook/segmentation_and_grasp-0c6e1a85689049d5b97d8bcce5dc585d#265f9cac60ce4aa3ac9e80a11a44fdbf


import os
from copy import deepcopy
from urllib.request import urlretrieve

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.utils.data
import torchvision
import torchvision.transforms.functional as Tf
from IPython.display import clear_output, display
from pydrake.all import (BaseField, Concatenate, Fields, MeshcatVisualizer,
                         MeshcatVisualizerParams, PointCloud, Quaternion, Rgba,
                         RigidTransform, RotationMatrix, StartMeshcat)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from torchvision.models.detection import MaskRCNN_ResNet50_FPN_Weights
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor

from manipulation.clutter import GenerateAntipodalGraspCandidate
from manipulation.scenarios import AddRgbdSensors
from manipulation.utils import AddPackagePaths, LoadDataResource

from lang_sam import LangSAM

mustard_ycb_idx = 3

def get_instance_segmentation_model(num_classes):
    # load an instance segmentation model pre-trained on COCO
    model = torchvision.models.detection.maskrcnn_resnet50_fpn(
        weights=MaskRCNN_ResNet50_FPN_Weights.DEFAULT, progress=False)

    # get the number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    # replace the pre-trained head with a new one
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

    # now get the number of input features for the mask classifier
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    # and replace the mask predictor with a new one
    model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask,
                                                    hidden_layer,
                                                    num_classes)
    
    return model


def setup_model(num_classes):
    model = get_instance_segmentation_model(num_classes)
    device = torch.device(
        'cuda') if torch.cuda.is_available() else torch.device('cpu')
    # model.load_state_dict(torch.load('clutter_maskrcnn_model.pt'))
    #model.load_state_dict(torch.load('maskrcnn_resnet50_fpn_coco-bf2d0c1e.pth'))

    model.eval()

    model.to(device)

    return model, device


def get_lang_sam(sam_type):
    return LangSAM(sam_type)





