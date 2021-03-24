
"""
Simple Inference Script of EfficientDet-Pytorch
"""
import time
import torch
from torch.backends import cudnn
from model.efficientdet.backbone import EfficientDetBackbone
from model.efficientdet.utils import BBoxTransform, ClipBoxes
from utils import postprocess, STANDARD_COLORS, standard_to_bgr
from dataloader.freicar_dataloader import FreiCarDataset
from torch.utils.data import DataLoader
from model.efficientdet.dataset import collater
from utils import display
import argparse
import cv2
import numpy as np
#from dataloader.run_freicar_dataloader import GenBBOverlay
########################################################################
# Object Detection testing script
# Modified by: Jannik Zuern (zuern@informatik.uni-freiburg.de)
########################################################################


ap = argparse.ArgumentParser()
ap.add_argument('-w', '--weights', type=str, default=None, help='/path/to/weights')
args = ap.parse_args()


compound_coef = 0  # whether you use D-0, D-1, ... etc
force_input_size = None  # set None to use default size

# replace this part with your project's anchor config
anchor_ratios = [(1.0, 1.0), (1.4, 0.7), (0.7, 1.4)]
anchor_scales = [2 ** 0, 2 ** (1.0 / 3.0), 2 ** (2.0 / 3.0)]

threshold = 0.2
iou_threshold = 0.2

use_cuda = True
cudnn.fastest = True
cudnn.benchmark = True

obj_list = ['freicar']

val_set = FreiCarDataset(data_dir="./dataloader/data/",
                         padding=(0, 0, 12, 12),
                         split='validation',
                         load_real=False)

val_params = {'batch_size': 1,
              'shuffle': False,
              'drop_last': True,
              'collate_fn': collater,
              'num_workers': 1}

val_generator = DataLoader(val_set, **val_params)

color_list = standard_to_bgr(STANDARD_COLORS)


model = EfficientDetBackbone(compound_coef=compound_coef,
                             num_classes=len(obj_list),
                             ratios=anchor_ratios,
                             scales=anchor_scales)

model.load_state_dict(torch.load(args.weights, map_location='cpu'))

model.requires_grad_(False)
model.eval()

def TensorImage3ToCV(data):
    #cv = np.transpose(data.squeeze(), (2, 1, 0))
    cv = data.squeeze()
    cv = cv2.cvtColor(cv, cv2.COLOR_RGB2BGR)
    return cv


def GenBBOverlay(bbs, draw_image=None):
    bb_image = np.zeros((draw_image.shape[0], draw_image.shape[1], 3)).astype(np.uint8)
    overlay_img = (draw_image.copy()).astype(np.uint8)
    if draw_image is not None:
        cv2.rectangle(draw_image, (bbs[0], bbs[1]), ( bbs[2],bbs[3]), color=(255, 0, 0), thickness=2)
    overlay = cv2.addWeighted(bb_image, 0.3, overlay_img, 0.7, 0)
    return overlay

if use_cuda:
    model = model.cuda()

# Iterate over validation set and display predictions
####### Made by Davide Rezzoli
for iter, data in enumerate(val_generator):

    imgs = data['img'].cuda()
    annot = data['annot'].cuda()

    with torch.no_grad():
        features, regression, classification, anchors = model(imgs)

    regressBoxes = BBoxTransform()
    clipBoxes = ClipBoxes()

    predictions = postprocess(imgs, anchors, regression, classification,
                              regressBoxes, clipBoxes, threshold, iou_threshold)
    imgs = imgs.permute(0, 2, 3, 1).cpu().numpy()
    gt_img = TensorImage3ToCV(imgs)

    bbox = list(np.array(data["annot"].squeeze()).astype(int))

    if len(bbox) == 2:
        for box in bbox:
            overlay =GenBBOverlay(box, gt_img)
    else:
        overlay = GenBBOverlay(bbox, gt_img)
    gt_img = np.expand_dims(gt_img, axis=0)
    display(predictions, gt_img, imshow=True, imwrite=False, obj_list=obj_list)


print('Running speed test...')
print('inferring image for 100 times...')

n_iterations = 100


# get first batch of dataloader
data = next(iter(val_generator))

# move to GPU
imgs = data['img'].cuda()
annot = data['annot'].cuda()

t1 = time.time()

for _ in range(n_iterations):
    with torch.no_grad():
        _, regression, classification, anchors = model(imgs)

    regressBoxes = BBoxTransform()
    clipBoxes = ClipBoxes()

    out = postprocess(imgs,
                      anchors, regression, classification,
                      regressBoxes, clipBoxes,
                      threshold, iou_threshold)

t2 = time.time()
tact_time = (t2 - t1) / n_iterations

print('\n\nBenchmark result:')
print('Average inference time on 1 image: {:f} s'.format(tact_time))
print('Average FPS: {:f} @batch_size 1'.format(1 / tact_time))

