
########################################################################
# Object Detection training script
# Modified by: Jannik Zuern (zuern@informatik.uni-freiburg.de)
########################################################################

import argparse
import os
import numpy as np
import torch
import yaml
from torch.utils.data import DataLoader
from tqdm.autonotebook import tqdm
from model.efficientdet.backbone import EfficientDetBackbone
from model.efficientdet.dataset import collater
from model.efficientdet.loss import FocalLoss
from utils import CustomDataParallel, get_last_weights, init_weights, boolean_string
from dataloader.freicar_dataloader import FreiCarDataset
from utils import VisdomLinePlotter, visualize_imgs_with_gt_bbox


class Params:
    def __init__(self, project_file):
        self.params = yaml.safe_load(open(project_file).read())

    def __getattr__(self, item):
        return self.params.get(item, None)


def get_args():
    parser = argparse.ArgumentParser('Yet Another EfficientDet Pytorch: SOTA object detection network - Zylo117')
    parser.add_argument('-p', '--project', type=str, default='freicar-detection', help='project file that contains parameters')
    parser.add_argument('-c', '--compound_coef', type=int, default=0, help='coefficients of efficientdet')
    parser.add_argument('-n', '--num_workers', type=int, default=8, help='num_workers of dataloader')
    parser.add_argument('--batch_size', type=int, default=12, help='The number of images per batch among all devices')
    parser.add_argument('--lr', type=float, default=1e-4)
    parser.add_argument('--num_epochs', type=int, default=100)
    parser.add_argument('--val_interval', type=int, default=1, help='Number of epoches between valing phases')
    parser.add_argument('--save_interval', type=int, default=500, help='Number of steps between saving')
    parser.add_argument('--es_min_delta', type=float, default=0.0, help='Early stopping\'s parameter: minimum change loss to qualify as an improvement')
    parser.add_argument('--es_patience', type=int, default=0,
                        help='Early stopping\'s parameter: number of epochs with no improvement after which training will be stopped. Set to 0 to disable this technique.')
    parser.add_argument('-w', '--load_weights', type=str, default=None,
                        help='whether to load weights from a checkpoint, set None to initialize, set \'last\' to load last checkpoint')
    parser.add_argument('--saved_path', type=str, default='logs/')

    # Visualization
    parser.add_argument('--visualize_gt', type=boolean_string, default=False,
                        help='whether to visualize the GT bounding boxes in the training loop/')

    args = parser.parse_args()
    return args


def train(opt):

    global plotter
    plotter = VisdomLinePlotter(env_name='FreiCar Object Detection')

    params = Params(f'projects/{opt.project}.yml')

    if params.num_gpus == 0:
        os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

    if torch.cuda.is_available():
        torch.cuda.manual_seed(42)
    else:
        torch.manual_seed(42)

    opt.saved_path = opt.saved_path + f'/{params.project_name}/'
    os.makedirs(opt.saved_path, exist_ok=True)

    # define paramteters for model training
    training_params = {'batch_size': opt.batch_size,
                       'shuffle': True,
                       'drop_last': True,
                       'collate_fn': collater,
                       'num_workers': opt.num_workers}

    # define paramteters for model evaluation
    val_params = {'batch_size': opt.batch_size,
                  'shuffle': False,
                  'drop_last': True,
                  'collate_fn': collater,
                  'num_workers': opt.num_workers}

    # get training dataset
    training_set = FreiCarDataset(data_dir="./dataloader/data/",
                                 padding=(0, 0, 12, 12),
                                 split='training',
                                 load_real=True)

    # and make data generator from dataset
    training_generator = DataLoader(training_set, **training_params)

    # get validation dataset
    val_set = FreiCarDataset(data_dir="./dataloader/data/",
                            padding=(0, 0, 12, 12),
                            split='validation',
                            load_real=False)

    # and make data generator from dataset
    val_generator = DataLoader(val_set, **val_params)

    # Instantiation of the EfficientDet model
    model = EfficientDetBackbone(num_classes=len(params.obj_list),
                                 compound_coef=opt.compound_coef,
                                 ratios=eval(params.anchors_ratios),
                                 scales=eval(params.anchors_scales))

    # load last weights if training from checkpoint
    if opt.load_weights is not None:
        if opt.load_weights.endswith('.pth'):
            weights_path = opt.load_weights
        else:
            weights_path = get_last_weights(opt.saved_path)
        try:
            last_step = int(os.path.basename(weights_path).split('_')[-1].split('.')[0])
        except:
            last_step = 0

        try:
            ret = model.load_state_dict(torch.load(weights_path), strict=False)
        except RuntimeError as e:
            print(f'[Warning] Ignoring {e}')
            print('[Warning] Don\'t panic if you see this, this might be because you load a pretrained weights with '
                'different number of classes. The rest of the weights should be loaded already.')

        print(f'[Info] loaded weights: {os.path.basename(weights_path)}, resuming checkpoint from step: {last_step}')
    else:
        last_step = 0
        print('[Info] initializing weights...')
        init_weights(model)

    if params.num_gpus > 0:
        model = model.cuda()
        if params.num_gpus > 1:
            model = CustomDataParallel(model, params.num_gpus)

    optimizer = torch.optim.AdamW(model.parameters(), opt.lr)

    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=3, verbose=True)

    best_loss = 1e5
    best_epoch = 0
    step = max(0, last_step)

    # Define training criterion
    criterion = FocalLoss()

    # Set model to train mode
    model.train()

    num_iter_per_epoch = len(training_generator)

    print('Started Training')

    # Train loop
    for epoch in range(opt.num_epochs):
        last_epoch = step // num_iter_per_epoch
        if epoch < last_epoch:
            continue

        epoch_loss = []  # here we append new total losses for each step

        progress_bar = tqdm(training_generator)
        for iter, data in enumerate(progress_bar):
            if iter < step - last_epoch * num_iter_per_epoch:
                progress_bar.update()
                continue

            ##########################################
            # TODO: implement me!
            # Made by DavideRezzoli
            ##########################################
            optimizer.zero_grad()
            _, reg, clas, anchor = model(data['img'].cuda())
            cls_loss, reg_loss = criterion(clas, reg, anchor, data['annot'].cuda())
            loss = cls_loss.mean() + reg_loss.mean()
            loss.backward()
            optimizer.step()



            epoch_loss.append(float(loss))

            progress_bar.set_description(
                'Step: {}. Epoch: {}/{}. Iteration: {}/{}. Cls loss: {:.5f}. Reg loss: {:.5f}. Total loss: {:.5f}'.format(
                    step, epoch, opt.num_epochs, iter + 1, num_iter_per_epoch, cls_loss.item(),
                    reg_loss.item(), loss.item()))

            plotter.plot('Total loss', 'train', 'Total loss', step, loss.item())
            plotter.plot('Regression_loss', 'train', 'Regression_loss', step, reg_loss.item())
            plotter.plot('Classfication_loss', 'train', 'Classfication_loss', step, cls_loss.item())

            # log learning_rate
            current_lr = optimizer.param_groups[0]['lr']
            plotter.plot('learning rate', 'train', 'Classfication_loss', step, current_lr)

            # increment step counter
            step += 1

            if step % opt.save_interval == 0 and step > 0:
                save_checkpoint(model, f'efficientdet-d{opt.compound_coef}_{epoch}_{step}.pth')
                print('saved checkpoint...')

        # adjust learning rate via learning rate scheduler
        scheduler.step(np.mean(epoch_loss))

        if epoch % opt.val_interval == 0:

            print('Evaluating model')

            model.eval()
            loss_regression_ls = []  # here we append new regression losses for each step
            loss_classification_ls = [] # here we append new classification losses for each step

            for iter, data in enumerate(val_generator):

                with torch.no_grad():
                    ##########################################
                    # TODO: implement me!
                    # Made by Davide Rezzoli
                    #########################################
                    _, reg, clas, anchor = model(data['img'].cuda())
                    cls_loss, reg_loss = criterion(clas, reg, anchor, data['annot'].cuda())

                    loss_classification_ls.append(cls_loss.item())
                    loss_regression_ls.append(reg_loss.item())

                    cls_loss = np.mean(loss_classification_ls)
                    reg_loss = np.mean(loss_regression_ls)
                    loss = cls_loss + reg_loss

            # LOGGING
            print(
                'Val. Epoch: {}/{}. Classification loss: {:1.5f}. Regression loss: {:1.5f}. Total loss: {:1.5f}'.format(
                    epoch, opt.num_epochs, cls_loss, reg_loss, loss))

            plotter.plot('Total loss', 'val', 'Total loss', step, loss.item())
            plotter.plot('Regression_loss', 'val', 'Regression_loss', step, reg_loss.item())
            plotter.plot('Classfication_loss', 'val', 'Classfication_loss', step, cls_loss.item())

            # Save model checkpoint if new best validation loss
            if loss + opt.es_min_delta < best_loss:
                best_loss = loss
                best_epoch = epoch

                save_checkpoint(model, f'efficientdet-d{opt.compound_coef}_{epoch}_{step}.pth')

            model.train()

            # Early stopping
            if epoch - best_epoch > opt.es_patience > 0:
                print('[Info] Stop training at epoch {}. The lowest loss achieved is {}'.format(epoch, best_loss))
                break


def save_checkpoint(model, name):
    if isinstance(model, CustomDataParallel):
        torch.save(model.module.model.state_dict(), os.path.join(opt.saved_path, name))
    else:
        torch.save(model.state_dict(), os.path.join(opt.saved_path, name))


if __name__ == '__main__':
    opt = get_args()
    train(opt)
