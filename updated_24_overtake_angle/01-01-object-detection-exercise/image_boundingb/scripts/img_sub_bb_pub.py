#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
# from raiscar_msgs.msg import ControlCommand


import argparse
import torch
import yaml
# from tqdm import tqdm

import sys
sys.path.append('/home/freicar/freicar_ws/src/freicar_base/freicar_exercises/01-01-object-detection-exercise/model/efficientdet')

# print(os.getcwd())

from pathlib import *
Path.cwd()
a = Path.home()


import pdb; pdb.set_trace()
from model.efficientdet.backbone import EfficientDetBackbone
from model.efficientdet.utils import BBoxTransform, ClipBoxes
from utils import postprocess, boolean_string
from dataloader.freicar_dataloader import FreiCarDataset
from model.efficientdet.dataset import collater
from torch.utils.data import DataLoader
import numpy as np
# from mean_average_precision import MetricBuilder
from std_msgs.msg import String
import torchvision.transforms.functional as TF

from beginner_tutorials.msg import bb

ap = argparse.ArgumentParser()
ap.add_argument('-p', '--project', type=str, default='freicar-detection', help='project file that contains parameters')
ap.add_argument('-c', '--compound_coef', type=int, default=0, help='coefficients of efficientdet')
ap.add_argument('-w', '--weights', type=str, default='logs/efficientdet-d0_95_166000.pth', help='/path/to/weights')
ap.add_argument('--nms_threshold', type=float, default=0.5,
                help='nms threshold, don\'t change it if not for testing purposes')
ap.add_argument('--cuda', type=boolean_string, default=True)
ap.add_argument('--device', type=int, default=0)
args = ap.parse_args()

compound_coef = args.compound_coef
nms_threshold = args.nms_threshold
use_cuda = args.cuda
gpu = args.device

project_name = args.project
weights_path = args.weights

params = yaml.safe_load(open(f'projects/freicar-detection.yml'))
obj_list = params['obj_list']

threshold = 0.2
iou_threshold = 0.5

# test_image = Image.open(test_image_name).convert('RGB')test_image = Image.open(test_image_name).convert('RGB')
def callback(msg):
    np_img = np.fromstring(msg.data, dtype=np.uint8).reshape((720, 1280, 3))
    np_img = np_img[:, :, :3]
    img_tensor = TF.to_tensor(np_img)
    img_tensor = TF.resize(img_tensor,[384,640])
    img_tensor.unsqueeze_(0)
    img_tensor = img_tensor.cuda()

    model = EfficientDetBackbone(compound_coef=compound_coef, num_classes=len(obj_list),ratios=eval(params['anchors_ratios']), scales=eval(params['anchors_scales']))

    model.load_state_dict(torch.load(weights_path, map_location=torch.device('cpu')))

    model.requires_grad_(False)
    model.eval()
    if use_cuda:
        model = model.cuda()
    print("image data type",type(msg.data))
    with torch.no_grad():
        features, regression, classification, anchors = model(img_tensor)
    regressBoxes = BBoxTransform()
    clipBoxes = ClipBoxes()
    preds = postprocess(img_tensor, anchors, regression, classification, regressBoxes, clipBoxes, threshold,
                        nms_threshold)
    preds = preds[0]

    bb_msg = preds['rois']
    # pub = rospy.Publisher('/freicar_1/control', ControlCommand, queue_size=10)
    pub = rospy.Publisher('/freicar_1/bounding_box', bb, queue_size=10)
    # rospy.spin()
    msg_to_publish = bb()

    rate = rospy.Rate(10)  # 10hz
    # import pdb;    pdb.set_trace()
    while not rospy.is_shutdown():
        msg_to_publish.x1 = bb_msg[0][0]
        msg_to_publish.x2 = bb_msg[0][1]
        msg_to_publish.y1 = bb_msg[0][2]
        msg_to_publish.y2 = bb_msg[0][3]

        # import pdb; pdb.set_trace()
        pub.publish(msg_to_publish)
        rate.sleep()



    bgr = np.zeros((np_img.shape[0], np_img.shape[1], 3), dtype=np.uint8)
    cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR, bgr, 3)
    cv2.imshow('RGB image', bgr)
    cv2.waitKey(10)

def listener():
    rospy.init_node('listener', anonymous=True)
    img_sub = rospy.Subscriber('/freicar_1/sim/camera/rgb/front/image', Image, callback, queue_size=10)
    # pub = rospy.Publisher('/freicar_1/bounding_box', bb, queue_size=10)
    rospy.spin()



if __name__ == '__main__':

    listener()

