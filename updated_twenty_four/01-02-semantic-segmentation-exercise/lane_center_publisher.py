import argparse
import torch
import torch.nn.parallel
import torch.optim
import torch.utils.data
import torch.utils.data.distributed
from model import fast_scnn_model

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import torchvision.transforms.functional as TF

import os
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import birdsEyeT

import pdb

a = os.getcwd()
parser = argparse.ArgumentParser(description='Segmentation and Regression Training')
parser.add_argument('--resume', default=a+'/checkpoint.pth.tar', type=str, metavar='PATH',
                    help='path to latest checkpoint (default: none)')
parser.add_argument('--start_epoch', default=0, type=int, help="Start at epoch X")
parser.add_argument('--batch_size', default=1, type=int, help="Batch size for training")
parser.add_argument('-e', '--evaluate', dest='evaluate', action='store_true',
                    help='evaluate model on validation set')
parser.add_argument('--print-freq', '-p', default=10, type=int,
                    metavar='N', help='print frequency (default: 10)')
best_iou = 0
args = parser.parse_args()
model = fast_scnn_model.Fast_SCNN(3, 4)
model = model.cuda()
model.train()

pub = rospy.Publisher('/freicar_1/seg_classes', Image, queue_size=10)
pub2 = rospy.Publisher('/freicar_1/seg_lanes', Image, queue_size=10)
topic = 'freicar_1/lane_center_points'
pub3 = rospy.Publisher(topic, MarkerArray, queue_size=10)


markerArray = MarkerArray()

count = 0
MARKERS_MAX = 10000

if args.resume:
    if os.path.isfile(args.resume):
        print("=> loading checkpoint '{}'".format(args.resume))
        model.load_state_dict(torch.load(args.resume)['state_dict'])
        args.start_epoch = 0


    else:
        print("=> no checkpoint found at '{}'".format(args.resume))
else:
    args.start_epoch = 0

def visJetColorCoding(name, img):
    img = img.detach().cpu().squeeze().numpy()
    color_img = np.zeros(img.shape, dtype=img.dtype)
    cv2.normalize(img, color_img, 0, 255, cv2.NORM_MINMAX)
    color_img = color_img.astype(np.uint8)
    color_img = cv2.applyColorMap(color_img, cv2.COLORMAP_JET, color_img)
    return color_img

def TensorImage1ToCV(data):
   cv = data.cpu().data.numpy().squeeze()
   return cv

def visImage3Chan(data, name):
    cv = np.transpose(data.cpu().data.numpy().squeeze(), (1, 2, 0))
    cv = cv2.cvtColor(cv, cv2.COLOR_RGB2BGR)
    cv2.imshow(name, cv)

def callback(msg):

    np_img = np.fromstring(msg.data, dtype=np.uint8).reshape((720, 1280, 3))
    np_img = np_img[:, :, :3]
    img_tensor = TF.to_tensor(np_img)
    img_tensor = TF.resize(img_tensor,[384,640])
    img_tensor.unsqueeze_(0)
    img_tensor = img_tensor.cuda()                                   # np.shape torch.Size([1, 3, 384, 640])

    seg_cla, seg_reg = model(img_tensor)                             # np.shape torch.Size([1, 4, 384, 640])

    seg_cla2 = seg_cla.cpu().detach().numpy()
    seg_classes = np.argmax(seg_cla2, axis=1)
    seg_classes = seg_classes[0,:,:]                                 # shape is now (384,640)

    pred = torch.argmax(seg_cla, 1)
    bridge = CvBridge()
    rate = rospy.Rate(10)
    color_img = visJetColorCoding("img", pred[0].float())
    image_message = bridge.cv2_to_imgmsg(color_img)
    seg_reg = seg_reg.cpu().detach().numpy()
    seg_lanes = seg_reg[0, 0, :, :]                                  # shape is now (384,640)
    lanes_msg = bridge.cv2_to_imgmsg(seg_lanes)

    pub.publish(image_message)
    pub2.publish(lanes_msg)

    hom_conv = birdsEyeT.birdseyeTransformer('dataset_helper/freicar_homography.yaml', 3, 3, 200, 2)  # 3mX3m, 200 pixel per meter            # import path
    bev_lanes = hom_conv.birdseye(seg_lanes)  # 600x600

    lanes_nor = np.zeros(bev_lanes.shape, dtype=bev_lanes.dtype)
    cv2.normalize(bev_lanes, lanes_nor, 0, 255, cv2.NORM_MINMAX)

    counter = 0
    counter_id = 0

    N_max = 700
    random_indices = np.random.randint(1000, size=(N_max))
    random_indices_set = set(random_indices)


    for i in range(np.shape(bev_lanes)[0]):

        for j in range(np.shape(bev_lanes)[1]):
            if lanes_nor[i, j] > 150:
                if counter in random_indices_set:
                    counter_id = counter_id + 1
                    marker = Marker()
                    marker.header.frame_id = "/freicar_1/base_link"
                    marker.id = counter_id
                    marker.type = marker.SPHERE
                    marker.action = marker.ADD
                    marker.scale.x = 0.01
                    marker.scale.y = 0.01
                    marker.scale.z = 0.0001
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.pose.orientation.w = 1.0
                    marker.pose.position.x = i/ 100.0
                    marker.pose.position.y = (j / 100.0) - (600.0 / 200.0)
                    marker.pose.position.z = 0.0
                    markerArray.markers.append(marker)
                    print("added")

                counter = counter + 1

    print(counter)

    pub3.publish(markerArray)

    rate.sleep()


def subscriber_publisher():
    rospy.init_node('seg_reg_publisher', anonymous=True)
    img_sub = rospy.Subscriber('/freicar_1/sim/camera/rgb/front/image', Image, callback, queue_size=10)
    rospy.spin()



if __name__ == '__main__':
    subscriber_publisher()
