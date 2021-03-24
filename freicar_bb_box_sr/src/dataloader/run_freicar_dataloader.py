import freicar_dataloader as loader
import torch
import cv2
import numpy as np


########################################################################
# Demo test script for the freicar dataloader for bounding boxes
# Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
########################################################################

def GenBBOverlay(bbs, draw_image=None):
    bb_image = np.zeros((draw_image.shape[0], draw_image.shape[1], 3)).astype(np.uint8)
    overlay_img = (draw_image.copy()).astype(np.uint8)
    for car in bbs.values():
        for bb in car:
            if draw_image is not None:
                cv2.rectangle(bb_image, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']),
                              color=(255, 0, 0), thickness=2)

    overlay = cv2.addWeighted(bb_image, 0.3, overlay_img, 0.7, 0)
    return overlay


def TensorImage3ToCV(data):
    cv = np.transpose(data.cpu().data.numpy().squeeze(), (1, 2, 0))
    cv = cv2.cvtColor(cv, cv2.COLOR_RGB2BGR)
    return cv


static_data = loader.FreiCarDataset("./data/",
                                    padding=(0, 0, 12, 12),
                                    split='training',
                                    load_real=True)

train_loader = torch.utils.data.DataLoader(static_data,
                                           batch_size=1,
                                           shuffle=True,
                                           num_workers=1,
                                           pin_memory=False, drop_last=True)


for nr, (sample) in enumerate(train_loader):
    cv_rgb = TensorImage3ToCV(sample['rgb'][0])
    overlay = GenBBOverlay(sample['bbs'], cv_rgb)
    cv2.imshow('RGB', cv_rgb)
    cv2.imshow('BB Overlay', overlay)
    cv2.waitKey()
