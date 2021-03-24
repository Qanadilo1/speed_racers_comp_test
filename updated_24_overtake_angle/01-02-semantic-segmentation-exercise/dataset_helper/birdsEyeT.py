import numpy as np
import cv2
import yaml
####################################################################################################
# Author : Johan Vertens
# Descriptions: This class computes a perspective inverse mapping.
#               It can warp a image from a normal ego view to a top-down view
####################################################################################################

class birdseyeTransformer:
    def __init__(self, homography_path, width=2, height=2.5, pix_res=500, homography_downsacling=1.0):
        self.pix_to_streetimage, self.streetimage_to_pix = self.getBirdsEyeT(homography_path, width, pix_res, homography_downsacling)
        self.width = width
        self.height = height
        self.pix_res = pix_res

    def getBirdsEyeT(self, path, width_m, pix_res, downscale_factor = 2.):
        # birdseye transform
        with open(path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        src = data['src']
        dst = data['dst']

        src = np.array(src).astype(np.float32) / downscale_factor
        dst = np.array(dst).astype(np.float32)

        # pixel per meter
        ipm_scale = pix_res
        ipm_street_img_offset = width_m / 2.

        src = src.astype(np.float32)

        shiftedDst = dst
        shiftedDst[:, 0] = shiftedDst[:, 0] + ipm_street_img_offset
        shiftedDst = shiftedDst * ipm_scale

        pix_to_streetimage = cv2.getPerspectiveTransform(src, shiftedDst)
        pix_to_streetimage = pix_to_streetimage.astype(np.float32)
        streetimage_to_pix = cv2.getPerspectiveTransform(shiftedDst, src)
        streetimage_to_pix = streetimage_to_pix.astype(np.float32)
        return pix_to_streetimage, streetimage_to_pix

    def birdseye(self, image, interpolation_mode=cv2.INTER_LINEAR):
        bev = cv2.warpPerspective(image, self.pix_to_streetimage, dsize=(int(self.width * self.pix_res), int(self.height * self.pix_res)), flags=interpolation_mode+cv2.WARP_FILL_OUTLIERS)
        return bev

    def reverse_birdseye(self, image, s, interpolation_mode=cv2.INTER_LINEAR):
        return cv2.warpPerspective(image, self.streetimage_to_pix, dsize=(s[1], s[0]), flags=interpolation_mode+cv2.WARP_FILL_OUTLIERS)