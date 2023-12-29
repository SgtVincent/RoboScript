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
from segment_anything import build_sam, SamPredictor

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

def show_predictions_with_masks(scores, boxes, classes, masks):
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

        m = masks[obj_ind]
        img = np.ones((m.shape[0], m.shape[1], 3))
        for i in range(3):
            img[:,:,i] = color_mask[i]
        ax.imshow(np.dstack((img, m*0.45)))

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
        sam_checkpoint = "src/grounding_model/glip_sam/sam_vit_h_4b8939.pth"
        # glip_checkpoint = "src/grounding_model/det_models/glip_tiny_model_o365_goldg_cc_sbu.pth"
        # glip_config_file = "src/grounding_model/glip/configs/glip_Swin_T_O365_GoldG.yaml"
        device = "cuda"
        cfg.local_rank = 0
        cfg.num_gpus = 1
        cfg.merge_from_file(glip_config_file)
        cfg.merge_from_list(["MODEL.WEIGHT", glip_checkpoint])
        cfg.merge_from_list(["MODEL.DEVICE", device])
        self.glip_demo = GLIPDemo(
                            cfg,
                            min_image_size=800,
                            confidence_threshold=0.6,
                            show_mask_heatmaps=False
                        )
        self.sam_predictor = SamPredictor(build_sam(checkpoint=sam_checkpoint).to(device=device))
        
        
    def parse_2d_bbox(self,image,text_prompt,show=True, index=0):
        bboxes=[]
        labels=[]   
        for i, object in enumerate(text_prompt):
            scores, boxes, names = self.glip_demo.inference_on_image(image, object)
            if len(boxes)>0:
                for i in range(len(boxes)):
                    bboxes.append([int(x) for x in boxes.tolist()[i]])
                    #bboxes.append((boxes.tolist()[0]))
                    labels.append(names[i])
                    # draw output image
        mapping = {}
        id_counter = {}
        detect_result = {}

        # print(bboxes, labels)
        for bbox, name in zip(bboxes, labels):
            if name in id_counter:
                id_counter[name] += 1
                name_with_id = f"{name}_{id_counter[name]}"
            else:
                id_counter[name] = 0
                name_with_id = f"{name}_0"
            if show:
                print('---------------show-------------------')
                plt.figure(figsize=(10, 10))
                plt.imshow(image)
                show_predictions([0.6], [bbox], [name_with_id])
                plt.axis('off')
                plt.savefig(f'/home/cuite/data/figures/fig_{name_with_id}_{index}.jpg')

            detect_result['bbox'] = bbox
            mapping[name_with_id] = detect_result
        
        return mapping
    
    def parse_2d_bbox_mask(self,image,text_prompt,show=True, index=0):
        labels=[]
        masks = []
        bboxes = []
        confidence = []
        for i, object in enumerate(text_prompt):
            scores, boxes, names = self.glip_demo.inference_on_image(image, object)
            if len(boxes)>0:
                print('object', object)
                image_rbg = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                self.sam_predictor.set_image(image_rbg)
                for i, box in enumerate(boxes):
                    transformed_box = self.sam_predictor.transform.apply_boxes_torch(box, image_rbg.shape[:2])
                    mask, _, _ = self.sam_predictor.predict_torch(
                        point_coords=None,
                        point_labels=None,
                        boxes=transformed_box.to("cuda"),
                        multimask_output=False,
                    )
                    masks.append(mask.squeeze(0).squeeze(0))
                    labels.append(names[i])
                    bboxes.append([int(x) for x in boxes.tolist()[i]])
                    print('bboxes', bboxes)
                    confidence.append(scores[i])

                    # draw output image
        mapping = {}
        id_counter = {}
        # print(bboxes, labels)
        for bbox, name, mask, score in zip(bboxes, labels, masks, confidence):
            detect_result = {}
            if name in id_counter:
                id_counter[name] += 1
                name_with_id = f"{name}_{id_counter[name]}"
            else:
                id_counter[name] = 0
                name_with_id = f"{name}_0"
            if show:
                print('---------------show-------------------')
                plt.figure(figsize=(10, 10))
                plt.imshow(image)
                show_predictions_with_masks([score], [bbox], [name_with_id], [mask.to('cpu')])
                plt.axis('off')
                plt.savefig(f'/home/cuite/data/figures/fig_{name_with_id}_{index}.jpg')

            detect_result['bbox'] = bbox
            detect_result['mask'] = mask
            detect_result['score'] = score
            detect_result['camera_id'] = index
            detect_result['fig_id'] = f'fig_{name_with_id}_{index}'
            mapping[name_with_id] = detect_result

        
        return mapping
    
    def query_2d_bbox_list(self, sensor_data, object_list, show=False):
        image_list = sensor_data['rgb_image_list']
        bbox_list=[]
        for image in image_list:
            bbox_list.append(self.parse_2d_bbox(image,object_list,show))
        return bbox_list
    
    def query_2d_bbox_list_with_idx(self, sensor_data, object_list, show=True):
        image_list = sensor_data['rgb_image_list']
        bbox_list=[]
        idx_dict = {}
        for i, image in enumerate(image_list):
            bbox = self.parse_2d_bbox_mask(image,object_list,show, index=i)
            for key, _ in bbox.items():
                idx_dict[key] = i
            bbox_list.append(bbox)
        return bbox_list
    
        
        
    
