import numpy as np

#################################################################
# AUTHOR: Johan Vertens (vertensj@informatik.uni-freiburg.de)
# DESCRIPTION: Simple color coder for semantic segmentation
##################################################################

class ColorCoder():
    def get_color_coding(self):
        coding = {}
        coding[0] = [0, 0, 0]
        coding[1] = [255, 0, 0]
        coding[2] = [0, 255, 0]
        coding[3] = [0, 0, 255]
        return coding

    def color_code_labels(self, net_out, argmax=True):
        if argmax:
            labels, indices = net_out.max(1)
            labels_cv = indices.cpu().data.numpy().squeeze()
        else:
            labels_cv = net_out.cpu().data.numpy().squeeze()

        h = labels_cv.shape[0]
        w = labels_cv.shape[1]

        color_coded = np.zeros((h, w, 3), dtype=np.uint8)

        for x in range(w):
            for y in range(h):
                color_coded[y, x, :] = self.color_coding[labels_cv[y, x]]

        return color_coded

    def __init__(self):
        super(ColorCoder, self).__init__()
        self.color_coding = self.get_color_coding()