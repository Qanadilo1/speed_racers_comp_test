import torch
import numpy as np

def collater(data):
    imgs = [s['rgb'][0] for s in data]
    annots = [s['bbs'] for s in data]

    # tensor list to tensor
    imgs = torch.stack(imgs, dim=0)

    # normalize
    imgs = imgs.type(torch.float32) / 255.

    annotations_list = []

    for a in annots:
        annotations = np.zeros((0, 5))

        for k in a:
            if 'freicar' in k:

                bbox = a[k][0]

                x1 = bbox['x']
                y1 = bbox['y']
                x2 = x1 + bbox['width']
                y2 = y1 + bbox['height']
                class_id = 0  # for all samples

                # format: [x1, y1, x2, y2, class_id]
                annotation = np.zeros((1, 5))
                annotation[0, 0] = x1
                annotation[0, 1] = y1
                annotation[0, 2] = x2
                annotation[0, 3] = y2
                annotation[0, 4] = class_id

                annotations = np.append(annotations, annotation, axis=0)

        annotations = torch.from_numpy(annotations)
        annotations_list.append(annotations)

    # copy back
    annots = annotations_list

    max_num_annots = max(annot.shape[0] for annot in annots)

    if max_num_annots > 0:
        annot_padded = torch.ones((len(annots), max_num_annots, 5)) * -1

        for idx, annot in enumerate(annots):
            if annot.shape[0] > 0:
                annot_padded[idx, :annot.shape[0], :] = annot
    else:
        annot_padded = torch.ones((len(annots), 1, 5)) * -1

    return {'img': imgs,
            'annot': annot_padded}
