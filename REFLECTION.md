# Traffic Light Detection Model

The traffic light detection model used for this project is taken from this [github link](https://github.com/alex-lechner/Traffic-Light-Classification).


## Model
The author of the repository has trained 3 Tensorflow models and compared them:

**SSD Inception V2 Coco (17/11/2017)** : This model is very fast but does not generalize well on different data

**SSD Inception V2 Coco (11/06/2017)** : This model is very fast but does not generalize well on different data

**Faster RCNN Inception V2 Coco (28/01/2018)** : This model provides good precision and generalization of different data but is slow

**Faster RCNN Resnet101 Coco (11/06/2017)** : This model is highly accurate but very slow

Finally, the **SSD Inception V2 Coco (17/11/2017)** was choosen as the model to use.

## Dataset
The dataset used for training has been collected from the Udacity Simulator and labelled by the author themselves.