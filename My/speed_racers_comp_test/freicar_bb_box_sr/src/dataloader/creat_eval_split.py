import freicar_dataloader as fl
import random

db_path = "/home/vertensj/datasets/augsim_final/dyn/aug/detection_data/"

syn_rgb_path_files = sorted(fl.searchForFiles('*.png', db_path + '/synth/'), key=lambda y: float(y.name.split('.')[0].replace('_', '.')))
syn_rgb_path_files = [str(p) for p in syn_rgb_path_files]

print('Total number of images: %d' % len(syn_rgb_path_files))

eval_files = random.choices(syn_rgb_path_files, k=2000)

with open(db_path + '/eval_names.txt', 'w') as the_file:
    for ef in eval_files:
        the_file.write(ef+'\n')

print('Picked %d random names as eval set' % len(eval_files))

