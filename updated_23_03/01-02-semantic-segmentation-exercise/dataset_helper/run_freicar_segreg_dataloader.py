import freicar_segreg_dataloader as loader
import torch
import cv2
import numpy as np

########################################################################
# Demo test script for the freicar dataloader for semantic segmentation
# and regression
# Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
########################################################################

def visJetColorCoding(img):
    color_img = np.zeros(img.shape, dtype=img.dtype)
    cv2.normalize(img, color_img, 0, 255, cv2.NORM_MINMAX)
    color_img = color_img.astype(np.uint8)
    color_img = cv2.applyColorMap(color_img, cv2.COLORMAP_JET, color_img)
    return color_img

def GenRegOverlay(reg, overlay_image):
    reg_color = visJetColorCoding(reg)
    overlay = cv2.addWeighted(reg_color, 0.3, overlay_image, 0.7, 0)
    return overlay

def TensorImage3ToCV(data):
    cv = np.transpose(data.cpu().data.numpy().squeeze(), (1, 2, 0))
    cv = cv2.cvtColor(cv, cv2.COLOR_RGB2BGR)
    return cv

def TensorImage1ToCV(data):
    cv = data.cpu().byte().data.numpy().squeeze()
    return cv

static_data = loader.FreiCarLoader("../../data/", padding=(0, 0, 0, 0),
                                   split='validation', load_real=False)

train_loader = torch.utils.data.DataLoader(static_data, batch_size=1, shuffle=True, num_workers=1,
                                           pin_memory=False, drop_last=False)

for nr, (sample) in enumerate(train_loader):
    cv_rgb = TensorImage3ToCV((sample['rgb'] * 255.).byte())
    cv_reg = TensorImage1ToCV(sample['reg']* 255)
    cv_seg = TensorImage1ToCV(sample['seg'])

    cv2.imshow('RGB', cv_rgb)
    cv2.imshow('Regression', visJetColorCoding(cv_reg))
    cv2.imshow('Segmentation', visJetColorCoding(cv_seg))
    # import pdb ; pdb.set_trace()
    cv2.imshow('Regression overlay', GenRegOverlay(cv_reg, cv_rgb))
    cv2.waitKey()