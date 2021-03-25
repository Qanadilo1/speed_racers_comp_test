import argparse
import torch
import torch.nn.parallel
import torch.optim
import torch.utils.data
import torch.utils.data.distributed
from std_msgs.msg import Header
import time

import pdb
import os
a = os.getcwd()
print(a)
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import torchvision.transforms.functional as TF
from past_exercise.model import fast_scnn_model
from past_exercise import birdsEyeT



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

pub = rospy.Publisher('/freicar_1/sim/camera/rgb/front/reg_bev', Image, queue_size=10)

count = 0
if args.resume:
    if os.path.isfile(args.resume):
        print("=> loading checkpoint '{}'".format(args.resume))
        model.load_state_dict(torch.load(args.resume)['state_dict'])
        args.start_epoch = 0


    else:
        print("=> no checkpoint found at '{}'".format(args.resume))
else:
    args.start_epoch = 0
def thresholdSampling(img, t):
    torch_thresholding = torch.nn.Threshold(t, 0)
    return torch_thresholding(img)


def visJetColorCoding(img):
    # img = img.detach().cpu().squeeze().numpy()
    img = img.squeeze()

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

    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'freicar_1/base_link'
    msg.header = header
    np_img = np.fromstring(msg.data, dtype=np.uint8).reshape((720, 1280, 3))
    np_img = np_img[:, :, :3]
    img_tensor = TF.to_tensor(np_img)
    #### FIX LATER
    img_tensor = TF.resize(img_tensor,[384,640])
    img_tensor.unsqueeze_(0)
    img_tensor = img_tensor.cuda()                                   # np.shape torch.Size([1, 3, 384, 640])

    hom_conv = birdsEyeT.birdseyeTransformer('past_exercise/freicar_homography.yaml', 3, 3, 200, 2)
    seg_cla, seg_reg = model(img_tensor)                             # np.shape torch.Size([1, 4, 384, 640])
    thresh = 200
    seg_reg = seg_reg.squeeze()
    # thresholded_seg_reg = thresholdSampling(seg_reg, threshold=threshold)
    if (torch.nonzero(seg_reg).shape[0]>= 0):
        seg_reg_thresh = TensorImage1ToCV(seg_reg)
        seg_reg_thresh_bev = hom_conv.birdseye(seg_reg_thresh)
    seg_reg_thresh_bev = TF.to_tensor(seg_reg_thresh_bev)
    # bird_seg_reg = bird_seg_reg.detach().numpy()
    seg_reg_thresh_bev_thresh = thresholdSampling(seg_reg_thresh_bev, thresh)
    seg_reg_thresh_bev_thresh = seg_reg_thresh_bev_thresh.unsqueeze_(0)
    seg_reg_thresh_bev_thresh = seg_reg_thresh_bev_thresh.numpy()
    seg_reg_thresh_bev_thresh = seg_reg_thresh_bev_thresh[0, 0, :, :]

    # image_temp = Image()
    # header = Header(stamp=rospy.Time.now())
    # header.frame_id = 'map'
    # image_temp.encoding = 'rgb8'
    # # print(imgdata)
    # # image_temp.is_bigendian=True
    # image_temp.header = header
    # image_temp.step = 1241 * 3

    # # pdb.set_trace()
    # for i in range(600):
    #     for j in range(600):
    #         if (thresholded_bird[i][j] > 0):
    #             print(thresholded_bird[i][j])


    # pdb.set_trace()
    bridge = CvBridge()
    # bird_seg_reg = bird_seg_reg.cpu().detach().numpy()
    # lanes_nor = np.zeros(bird_seg_reg.shape, dtype=bird_seg_reg.dtype)
    # cv2.normalize(bird_seg_reg, lanes_nor, 0, 255, cv2.NORM_MINMAX)
    # image_message = bridge.cv2_to_imgmsg(lanes_nor)
    image_message = bridge.cv2_to_imgmsg(seg_reg_thresh_bev_thresh)
    image_message.header = header
    image_message.header.frame_id = 'freicar_1/base_link'
    # pdb.set_trace()
    # print(bird_seg_reg)


    # for i in range(600):
    #     for j in range(600):
    #         if (image_message[i][j] > 0):
    #             print(image_message[i][j])

    rate = rospy.Rate(10)

# Comment starts
    #
    # bridge = CvBridge()
    # rate = rospy.Rate(10)
    #
    # seg_reg = seg_reg.cpu().detach().numpy()
    # seg_lanes = seg_reg[0, 0, :, :]                                  # shape is now (384,640)
    # lanes_msg = bridge.cv2_to_imgmsg(seg_lanes)
    #
    #
    # hom_conv = birdsEyeT.birdseyeTransformer('past_exercise/freicar_homography.yaml', 3, 3, 200, 2)  # 3mX3m, 200 pixel per meter            # import path
    # bev_lanes = hom_conv.birdseye(seg_lanes)  # 600x600
    # # bev_lanes = hom_conv.birdseye(seg_reg)  # 600x600
    #
    # lanes_nor = np.zeros(bev_lanes.shape, dtype=bev_lanes.dtype)
    # cv2.normalize(bev_lanes, lanes_nor, 0, 255, cv2.NORM_MINMAX)
    # cv2.threshold(lanes_nor,30,255,0,lanes_nor)
    # # cv2.threshold(lanes_nor,120,lanes_nor,)
    # # cv2.th
    # # lanes_nor = visJetColorCoding(bev_lanes)
    # # lanes_nor = lanes_nor.astype(np.uint8)
    # # lanes_nor = cv2.applyColorMap(lanes_nor, cv2.COLORMAP_JET, lanes_nor)
    #
    # image_message = bridge.cv2_to_imgmsg(lanes_nor)
    # # image_message = bridge.cv2_to_imgmsg(lanes_nor)

# comment ends
    
    pub.publish(image_message)
    # pdb.set_trace()
    rate.sleep()


def subscriber_publisher():
    rospy.init_node('bev_lanes_img', anonymous=True)
    img_sub = rospy.Subscriber('/freicar_1/sim/camera/rgb/front/image', Image, callback, queue_size=10)
    rospy.spin()



if __name__ == '__main__':
    subscriber_publisher()
