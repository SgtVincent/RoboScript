import os 
import numpy as np
import time 
from typing import List, Dict, Tuple
from .grounding_base import GroundingBase

import argparse
import os
import copy

import numpy as np
import json
import torch
from PIL import Image, ImageDraw, ImageFont

import cv2
import matplotlib.pyplot as plt
import sys
sys.path.append('.')

from maskrcnn_benchmark.config import cfg
from src.grounding_model.glip.predictor_glip import GLIPDemo

# TODO: import model dependencies 

def show_predictions(scores, boxes, classes):
    num_obj = len(scores)
    if num_obj == 0:
        return
    ax = plt.gca()
    ax.set_autoscale_on(False)
    colors = plt.cm.gist_rainbow(np.linspace(0, 1, num_obj))

    for obj_ind in range(num_obj):
        box = boxes[obj_ind]
        score = scores[obj_ind]
        name = classes[obj_ind]

        # color_mask = np.random.random((1, 3)).tolist()[0]
        color_mask = colors[obj_ind]

        # m = masks[obj_ind][0]
        # img = np.ones((m.shape[0], m.shape[1], 3))
        # for i in range(3):
        #     img[:,:,i] = color_mask[i]
        # ax.imshow(np.dstack((img, m*0.45)))

        x0, y0, w, h = box[0], box[1], box[2] - box[0], box[3] - box[1]
        ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor=color_mask, facecolor=(0, 0, 0, 0), lw=2))

        label = name + ': {:.2}'.format(score)
        ax.text(x0, y0, label, color=color_mask, fontsize='large', fontfamily='sans-serif')

class GroundingGLIP(GroundingBase):
    """
    Grounding environment with model on remote server.
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # TODO: load arguments from kwargs 
        self.load_model()
        
    def load_model(self):
        """
        Since the model is on remote server, we need to wait for the model to be ready.
        """
        glip_checkpoint = "src/grounding_model/det_models/glip_large_model.pth"
        glip_config_file = "src/grounding_model/glip/configs/glip_Swin_L.yaml"
        # glip_checkpoint = "src/grounding_model/det_models/glip_tiny_model_o365_goldg_cc_sbu.pth"
        # glip_config_file = "src/grounding_model/glip/configs/glip_Swin_T_O365_GoldG.yaml"
        cfg.local_rank = 0
        cfg.num_gpus = 1
        cfg.merge_from_file(glip_config_file)
        cfg.merge_from_list(["MODEL.WEIGHT", glip_checkpoint])
        cfg.merge_from_list(["MODEL.DEVICE", "cuda"])
        self.glip_demo = GLIPDemo(
                            cfg,
                            min_image_size=800,
                            confidence_threshold=0.6,
                            show_mask_heatmaps=False
                        )
        
    def parse_2d_bbox(self,image,text_prompt,show=False):
        bboxes=[]
        labels=[]
        for object in text_prompt:
            scores, boxes, names = self.glip_demo.inference_on_image(image, object)
            print(boxes, names)
            if len(boxes)>0:
                for i in range(len(boxes)):
                    bboxes.append([int(x) for x in boxes.tolist()[i]])
                    #bboxes.append((boxes.tolist()[0]))
                    labels.append(names[i])
                    # draw output image
            if show:
                plt.figure(figsize=(10, 10))
                plt.imshow(image)
                show_predictions(scores, boxes, names)
                plt.axis('off')
        mapping = {}
        id_counter = {}
        # print(bboxes, labels)
        for bbox, name in zip(bboxes, labels):
            if name in id_counter:
                id_counter[name] += 1
                name_with_id = f"{name}_{id_counter[name]}"
            else:
                id_counter[name] = 0
                name_with_id = f"{name}_0"

            mapping[name_with_id] = bbox
        
        return mapping
    
    def query_2d_bbox_list(self, sensor_data, object_list, show=False):
        image_list = sensor_data['rgb_image_list']
        bbox_list=[]
        for image in image_list:
            bbox_list.append(self.parse_2d_bbox(image,object_list,show))
        return bbox_list
        
        
    
