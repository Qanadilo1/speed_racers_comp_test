import torch.utils.data as data
import os
import os.path
import cv2
import torch
import yaml
from pathlib import Path
import requests
from zipfile import ZipFile
import sys
#from skimage import io
import numpy as np


########################################################################
# Demo freicar dataloader for bounding boxes
# Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
########################################################################
##Felix & Sakshi##
def augmentator(im, aug_prob):

    im_temp = im
    aug_val = np.random.rand()
    mode = np.random.randint(0, 3)

    if aug_val <= aug_prob:
        #Brightness modifier (red)
        if mode == 0 or mode == 2:

            br_dr=np.random.rand()
            # image brightness enhancer
            if br_dr >= 0.5:
                hsvImg = cv2.cvtColor(im_temp, cv2.COLOR_BGR2HSV)
                value = np.random.randint(10, 30)
                vValue = hsvImg[..., 2]
                im_temp[..., 2] = np.where((255 - vValue) < value, 255, vValue + value)

            else:
                factor = np.random.uniform(0.5, 0.9)
                hsvImg = cv2.cvtColor(im_temp, cv2.COLOR_BGR2HSV)
                # decreasing the V channel by a factor from the original
                im_temp[..., 2] = hsvImg[..., 2] * factor

        gauss_bilat = np.random.rand()
        if mode == 1 or mode == 2:

            # Blur image with gaussian blur
            if gauss_bilat>=0.5:
                x_min_blur = 2
                x_max_blur = 5
                y_min_blur = 3
                y_max_blur = 7

                x_blur = np.random.randint(x_min_blur, x_max_blur)
                y_blur = np.random.randint(y_min_blur, y_max_blur)

                im_temp = cv2.blur(im_temp, (x_blur, y_blur))  # The bigger, the more blur, because kernel is bigger 5-10 best results
                #im_temp = Image.fromarray(blur_img, mode='RGB')
                lol="gauss "+str(x_blur)+" "+str(y_blur)

            else:
                im_temp = cv2.bilateralFilter(im_temp,15,80,80)
                lol="bilateral"

        return [im_temp]

    else:
        return [im]
def searchForFiles(name, search_path):
    paths = []
    for path in Path(search_path).rglob(name):
        paths.append(path)
    return paths


def loadRGB(paths, size=None, pad_info=None):
    rgb_im = [cv2.imread(p) for p in paths]
    rgb_im = augmentator(rgb_im[0], 0.5)  # enty augmentation
    if size is not None:
        rgb_im = [cv2.resize(im, size) for im in rgb_im]
    rgb_im = [cv2.cvtColor(i, cv2.COLOR_BGR2RGB) for i in rgb_im]
    rgb_im = [(torch.from_numpy(im)).permute(2, 0, 1) for im in rgb_im]
    if pad_info is not None:
        rgb_im = [torch.nn.functional.pad(d, pad_info, 'constant', 0) for d in rgb_im]
    return rgb_im


def generatePaths(fnames, type):
    npaths = []
    for fn in fnames:
        npaths.append(fn.replace('synth', type))
    return npaths


class FreiCarDataset(data.Dataset):
    def downloadData(self, data_dir):

        if not os.path.exists(data_dir):
            os.mkdir(data_dir)
        link = 'http://aisdatasets.informatik.uni-freiburg.de/freicar/detection_data.zip'
        file_name = data_dir + "/detection_data.zip"

        with open(file_name, "wb") as f:
            print("Downloading %s" % file_name)
            response = requests.get(link, stream=True)
            total_length = response.headers.get('content-length')

            if total_length is None:  # no content length header
                f.write(response.content)
            else:
                dl = 0
                total_length = int(total_length)
                for data in response.iter_content(chunk_size=4096):
                    dl += len(data)
                    f.write(data)
                    done = int(50 * dl / total_length)
                    sys.stdout.write("\r[%s%s]" % ('=' * done, ' ' * (50 - done)))
                    sys.stdout.flush()

        print('Unzipping data...')
        with ZipFile(data_dir + '/detection_data.zip', 'r') as zipObj:
            # Extract all the contents of zip file in current directory
            zipObj.extractall(data_dir)

        print('Removing zipped file ....')
        os.remove(data_dir + '/detection_data.zip')

    def filterEvalFiles(self, all_paths, eval_paths):
        out = []
        for p in all_paths:
            if p.split('/')[-1] not in eval_paths:
                out.append(p)
        return out

    def filterTrainFiles(self, all_paths, eval_paths):
        out = []
        for p in all_paths:
            if p.split('/')[-1] in eval_paths:
                out.append(p)
        return out

    def __init__(self, data_dir, padding, split='training', load_real=False):
        db_path = data_dir + '/detection_data/'

        if not os.path.exists(db_path):
            self.downloadData(data_dir)

        self.db_path = db_path
        self.load_real = load_real

        eval_name_file = open(db_path + 'eval_names.txt', 'r')
        self.eval_files = eval_name_file.readlines()
        self.eval_files = [f.rstrip() for f in self.eval_files]
        self.eval_files = [f.split('/')[-1] for f in self.eval_files]

        self.rgb_path_files = sorted(searchForFiles('*.png', db_path + '/synth/'),
                                     key=lambda y: float(y.name.split('.')[0].replace('_', '.')))
        self.rgb_path_files = [str(p) for p in self.rgb_path_files]

        if split == 'training':
            self.rgb_path_files = self.filterEvalFiles(self.rgb_path_files, self.eval_files)
        elif split == 'validation':
            self.rgb_path_files = self.filterTrainFiles(self.rgb_path_files, self.eval_files)
        else:
            print('Split: %s not known...' % split)
        if self.load_real:
            self.real_rgb_paths = generatePaths(self.rgb_path_files, 'real')
            self.rgb_path_files.extend(self.real_rgb_paths)

        self.padding = padding

        self.info_file = searchForFiles('info.yaml', db_path + '/info/')[0]

        assert (self.rgb_path_files.__len__() > 0)

        with open(str(self.info_file)) as file:
            self.info = yaml.load(file, Loader=yaml.FullLoader)

        self.length = len(self.rgb_path_files)
        self.filenames = [os.path.basename(p).replace('.png', '') for p in self.rgb_path_files]
        print('Size of FreiCAR detection dataset, {} split: {} images'.format(split, self.length))

    def __getitem__(self, index):
        '''
             Note 1:
             The subsequently called collater (defined in model/efficientdet/dataloader.py) expects a dict as a return value
             from this function. The dict should have to following form:
             out_dict['rgb']: rgb image
             out_dict['bbs']: bounding boxes for this image as saved in self.info

             Note 2:
             We are padding the image with (padding_left, padding_right, padding_top, padding_bottom) = (0, 0, 12, 12)
             when loading the files from disk in order to resize them to (640 x 384), because EfficientDet-D0 expects
             image dimensions to be a multiple of 32. This is NOT reflected in the bounding box coordinates in the downloaded dataset.
             Thus, make sure that you add the correct offset to the bounding box coordinates for the padded images in this function.
        '''
        ##########################################
        # TODO: implement me!
        # Made by DavideRezzoli & FelixHanser
        ##########################################
        file_name = self.filenames[index]

        image_path = self.rgb_path_files[index]

        image = loadRGB([image_path], pad_info=self.padding)

        info = self.info[file_name]

        for el in info:
            for el2 in info[el]:
                el2['y'] = el2['y'] + 12

        sample = {'rgb': image, 'bbs': info}

        return sample

    def __len__(self):
        return len(self.rgb_path_files)

    def __repr__(self):
        return self.__class__.__name__ + ' (' + self.db_path + ')'
