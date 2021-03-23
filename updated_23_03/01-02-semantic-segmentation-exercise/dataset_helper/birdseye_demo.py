import birdsEyeT
import cv2
import numpy as np
import torch
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


def visJetColorCoding(img):
    color_img = np.zeros(img.shape, dtype=img.dtype)
    # cv2.imshow('show image step1', color_img)
    cv2.normalize(img, color_img, 0, 255, cv2.NORM_MINMAX)
    # cv2.imshow('show image step2', color_img)

    color_img = color_img.astype(np.uint8)
    # cv2.imshow('bev gray', img)
    print("image pixels", img)
    # import pdb; pdb.set_trace()
    color_img = cv2.applyColorMap(color_img, cv2.COLORMAP_JET, color_img)

    # cv2.imshow('show image step3', color_img)
    points_x = []
    points_y = []
    points_xm = []
    points_ym = []
    pixels = []
    counter = 0
    for i in range(600):
        for j in range(600):
            if img[i,j] < 255 and img[i,j] > 248:
                # print(img[i,j])
                points_x.append(i)
                points_xm.append(j/100.0)
                points_y.append(j)
                points_ym.append(i/100.0 - 640.0/200.0)

                counter = counter + 1
    metric_points = Marker()
    metric_points.header.frame_id = "/freicar_1/base_link"
    metric_points.pose = [points_xm[0],points_ym[0],0]
    metric_points.type = Marker.CUBE
    print("points",points_xm[0],points_ym[0] )
    import pdb; pdb.set_trace()

    return color_img

def TensorImage1ToCV(data):
    cv = data.cpu().data.numpy().squeeze()
    return cv

lane_reg_example = cv2.imread('../../data/seg_reg_data/lane_dt/1603450738_189883683.png', cv2.IMREAD_GRAYSCALE)            # numpy ndarray
test = cv2.imread('../../data/testtest.png')            # numpy ndarray

corr_rgb_image = cv2.imread('../../data/detection_data/synth/1603450738_189883683.png')                                    # numpy ndarray 360x640x3
lane_reg_example = torch.from_numpy(lane_reg_example).float().unsqueeze(0)                                                 # Torch Tensor 1x360x640
lane_reg_example2 = cv2.imread('../../data/seg_reg_data/lane_dt/1603450738_189883683.png', cv2.IMREAD_GRAYSCALE) #360x640


hom_conv = birdsEyeT.birdseyeTransformer('freicar_homography.yaml', 3, 3, 200, 2)  # 3mX3m, 200 pixel per meter            # import path

bev_grayscale = hom_conv.birdseye(TensorImage1ToCV(lane_reg_example)) #600x600
bev = visJetColorCoding(bev_grayscale) #600x600x3

# original = visJetColorCoding(TensorImage1ToCV(lane_reg_example)) #360x640x3
#
# cv2.imshow('BEV2', lane_reg_example2)
# import pdb; pdb.set_trace()
# cv2.imshow('BEV', bev)
# cv2.imshow('BEV GRAYSCALE', bev_grayscale)
# cv2.imshow('Original perspective', original)
# cv2.imshow('testtest', test)

# cv2.imshow('Corresponding RGB image', corr_rgb_image)

cv2.waitKey()


# import pdb; pdb.set_trace()
# cv2.destroyAllWindows()

# [400,200] 250