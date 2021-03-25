#!/usr/bin/env python
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
import pdb
import sys
sys.path.insert(0,'/home/freicar/freicar_ws/src/speed_racers_comp_test/freicar_bb_box_sr/src/')
from model.efficientdet.backbone import EfficientDetBackbone
#print("deos this work")
from model.efficientdet.utils import BBoxTransform, ClipBoxes
from utils import postprocess, boolean_string
from dataloader.freicar_dataloader import FreiCarDataset
from model.efficientdet.dataset import collater
from torch.utils.data import DataLoader
import numpy as np
# from mean_average_precision import MetricBuilder

from std_msgs.msg import String
import torchvision.transforms.functional as TF
from bounding_box import bounding_box as bb
from freicar_bb_box_sr.msg import bb
# from image_boundingb.msg import bb as bb_msg


'''
ap = argparse.ArgumentParser()
ap.add_argument('-p', '--project', type=str, default='freicar-detection', help='project file that contains parameters')
ap.add_argument('-c', '--compound_coef', type=int, default=0, help='coefficients of efficientdet')
ap.add_argument('-w', '--weights', type=str, default='logs/freicar-detection/efficientdet-d0_95_166000.pth', help='/path/to/weights')
ap.add_argument('--nms_threshold', type=float, default=0.5,
                help='nms threshold, don\'t change it if not for testing purposes')
ap.add_argument('--cuda', type=boolean_string, default=True)
ap.add_argument('--device', type=int, default=0)
args = ap.parse_args()
'''
compound_coef = 0
nms_threshold = 0.5
use_cuda = True
gpu = 0

project_name = 'freicar-detection'
weights_path = '/home/freicar/freicar_ws/src/speed_racers_comp_test/freicar_bb_box_sr/src/logs/freicar-detection/efficientdet-d0_95_166000.pth'
import os, sys
here = os.path.dirname(os.path.abspath('/home/freicar/freicar_ws/src/speed_racers_comp_test/freicar_bb_box_sr/src/projects/'))
sys.path.append(here)
#print(here)
filename = os.path.join(here, "projects/freicar-detection.yml")
params = yaml.safe_load(open(filename))
obj_list = params['obj_list']

threshold = 0.2
iou_threshold = 0.5

# test_image = Image.open(test_image_name).convert('RGB')test_image = Image.open(test_image_name).convert('RGB')
def callback(msg):
    np_img = np.fromstring(msg.data, dtype=np.uint8).reshape((720, 1280, 3))
    np_img = np_img[:, :, :3]
    img_tensor = TF.to_tensor(np_img)

    # np_img2 = img_tensor.numpy()
    # np_img2 = np.reshape(np_img2, (720,1280,-1))
    # np_img2 = np.resize(np_img2, (384, 640))
    # pdb.set_trace()

    img_tensor = TF.resize(img_tensor,[384,640])

    np_img2 = img_tensor.numpy()
    np_img2 = np.resize(np_img2, (384, 640))

    # pdb.set_trace()
    # pdb.set_trace()
    img_tensor.unsqueeze_(0)
    img_tensor = img_tensor.cuda()
    # pdb.set_trace()

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
    preds = postprocess(img_tensor, anchors, regression, classification, regressBoxes, clipBoxes, threshold,nms_threshold)
    preds = preds[0]

    bb_msg = preds['rois']
    if(preds['scores'][0] > 0.99 ):
        bb_msg = preds['rois']
    else:
        bb_msg = preds['rois']
        bb_msg[0][0] = 0.0
        bb_msg[0][1] = 0.0
        bb_msg[0][2] = 0.0
        bb_msg[0][3] = 0.0
        print("no bb")

    pub = rospy.Publisher('/freicar_1/bounding_box', bb, queue_size=10)
    # rospy.spin()
    msg_to_publish = bb()
    # type(img_tensor.cpu().numpy())

    rate = rospy.Rate(10)  # 10hz
    # import pdb;    pdb.set_trace()

    msg_to_publish.x1 = bb_msg[0][0]
    msg_to_publish.x2 = bb_msg[0][1]
    msg_to_publish.y1 = bb_msg[0][2]
    msg_to_publish.y2 = bb_msg[0][3]
    print(type(msg_to_publish.x1))

    # bridge = CvBridge()
    # image22 = bridge.imgmsg_to_cv2(img_tensor)
    # bb.add(np_img, msg_to_publish.x1, msg_to_publish.y1, msg_to_publish.x1+msg_to_publish.x2, msg_to_publish.y1 + msg_to_publish.y2, color="green")
    # pdb.set_trace()
    # cv2.rectangle(np_img, (msg_to_publish.x1, msg_to_publish.x2), (msg_to_publish.y1,msg_to_publish.y2), (0,255,0))


    cv2.rectangle(np_img2, (msg_to_publish.x1, msg_to_publish.x2),(msg_to_publish.y1, msg_to_publish.y2), (0, 255, 0))

    print("bounding boxes  = ", msg_to_publish.x1, msg_to_publish.x2,msg_to_publish.y1, msg_to_publish.y2)
    # pdb.set_trace()
    # cv2.imshow("Image", np_img2)

    # cv2.waitKey(1)
    # import pdb; pdb.set_trace()
    pub.publish(msg_to_publish)
    rate.sleep()

    # pdb.set_trace()
    # msg_to_publish.x1 = 0.0
    # msg_to_publish.x2 = 0.0
    # msg_to_publish.y1 = 0.0
    # msg_to_publish.y2 = 0.0

    # bgr = np.zeros((np_img.shape[0], np_img.shape[1], 3), dtype=np.uint8)
    # cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR, bgr, 3)
    # cv2.imshow('RGB image', bgr)
    # cv2.waitKey(10)

def subscriber_publisher():
    rospy.init_node('img_sub_bb_pub', anonymous=True)
    img_sub = rospy.Subscriber('/freicar_1/sim/camera/rgb/front/image', Image, callback, queue_size=10)
    # pub = rospy.Publisher('/freicar_1/bounding_box', bb, queue_size=10)
    rospy.spin()



if __name__ == '__main__':

    subscriber_publisher()


