#Training
For training complete the file train_segmentation.py. In order to successfully train the network you need to implement the training loop and implement the getItem function in the dataloader (dataset_helper/freicar_segreg_dataloader.py).

As soon you run the dataloader the first time it will start to download the training/evaluation data automatically.

Similar to the first exercise we provide a file named "run_freicar_dataloader.py" that you can leverage to test your dataloader.

If you like to monitor your training with an additional visualization framework we ask you to add the necessary code for visdom/wandb/tensorboard yourself.

#Segmentation/Regression Visualization
To ease the visualization of segmentation results we provide you the file "dataset_helper/color_coder.py" that provides a class, which converts network outputs to a color coded RGB image.
For lane regression visualization see the function "visJetColorCoding" in the training script.

#Evaluation
As stated in the exercise-pdf you need to implement the IoU calculation and evaluate the IoU scores on the evaluation and training set respectively every N epochs during training.

#Bird's Eye View
The file "birdsEyeT.py" provides the class "birdseyeTransformer" that can transform a image to the bird's eye view using inverse perspective mapping. An example of how to use it is provided with the file "birdseye_demo.py".
Note that the input image size must be 640x360. If you modify, pad or shift the image this function won't work.