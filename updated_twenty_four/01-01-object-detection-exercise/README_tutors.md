# Setup

Install all the python dependencies using pip:

```console
pip install -r requirements.txt
```

# Dataset

The dataset will be downloaded by calling the following commands

```console
cd dataloader
python run_freicar_dataloader.py
```

This downloads a `.zip` file and extracts it content in the `dataloader` directory.
After extraction is completed, the zipfile will be automatically removed.

# Training

First, in a separate console window, start a visdom server

```console
python -m visdom.server
```
With a browser, you can navigate to `http://localhost:8097` to look at your training stats.
If you see no data but your training is already running, select the `FreiCar Object Detection` environment 
at the top of the visdom webpage.




Now, in a different console window, you can start your training

```console
python train.py -c 0 -p freicar-detection --batch_size 8 --lr 1e-5
```
Change the batch size in case the model does not fit into GPU memory.





# Inference Script 


If you want to have a look at your model predictions, you can run
```
python inference.py -w /path/to/your/weights.pth
python inference.py -w logs/freicar-detection/efficientdet-d0_95_166000.pth
```

# Evaluation Script 

For evaluating the model and calculating the mean average precision, run:
```console
python evaluate.py -w /path/to/your/weights.pth
python evaluate.py -w logs/freicar-detection/efficientdet-d0_95_166000.pth
```



# ROS: Launch the FreiCARSim Environment 

For evaluating the model and calculating the mean average precision, run:
```console
roslaunch freicar_launch sim_base.launch
```