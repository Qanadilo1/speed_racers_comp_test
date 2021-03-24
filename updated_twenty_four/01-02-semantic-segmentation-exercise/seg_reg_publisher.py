import argparse
import torch
import torch.nn.parallel
import torch.optim
import torch.utils.data
import torch.utils.data.distributed
import pdb
from model import fast_scnn_model

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2
import numpy as np
import torchvision.transforms.functional as TF
import os



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


def callback(msg):

    np_img = np.fromstring(msg.data, dtype=np.uint8).reshape((720, 1280, 3))
    np_img = np_img[:, :, :3]
    img_tensor = TF.to_tensor(np_img)
    img_tensor = TF.resize(img_tensor,[384,640])
    img_tensor.unsqueeze_(0)
    img_tensor = img_tensor.cuda()                          # np.shape torch.Size([1, 3, 384, 640])


    seg_cla, seg_reg = model(img_tensor)                                 # np.shape torch.Size([1, 4, 384, 640])

    seg_cla2 = seg_cla.cpu().detach().numpy()
    seg_classes = np.argmax(seg_cla2, axis=1)
    seg_classes = seg_classes[0,:,:]      # shape is now (384,640)

    pred = torch.argmax(seg_cla, 1)
    bridge = CvBridge()
    rate = rospy.Rate(10)
    color_img = visJetColorCoding("img", pred[0].float())
    image_message = bridge.cv2_to_imgmsg(color_img)

    cv2.imshow("classes", color_img)
    cv2.waitKey(1)
    # pdb.set_trace()
    # cv2.imshow("classes", color_img)
    seg_reg = seg_reg.cpu().detach().numpy()
    seg_lanes = seg_reg[0, 0, :, :]  # shape is now (384,640)
    lanes_msg = bridge.cv2_to_imgmsg(seg_lanes)


    pub = rospy.Publisher('/freicar_1/seg_classes', Image, queue_size=10)
    pub2 = rospy.Publisher('/freicar_1/seg_lanes', Image, queue_size=10)
    pub.publish(image_message)
    pub2.publish(lanes_msg)

    rate.sleep()


def subscriber_publisher():
    rospy.init_node('seg_reg_publisher', anonymous=True)
    img_sub = rospy.Subscriber('/freicar_1/sim/camera/rgb/front/image', Image, callback, queue_size=10)
    rospy.spin()



if __name__ == '__main__':
    subscriber_publisher()

