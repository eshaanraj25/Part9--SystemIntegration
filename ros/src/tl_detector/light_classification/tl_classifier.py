from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime

class TLClassifier(object):
    def __init__(self):
        PATH_TO_GRAPH = r'light_classification/model/ssd_inception.pb'

        # Load Tensorflow Graph
        # And initialize variables
        self.graph = tf.Graph()
        self.threshold = 0.5

        with self.graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as f:
                graph_def.ParseFromString(f.read())
                tf.import_graph_def(graph_def, name = '')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

        self.session = tf.Session(graph = self.graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Infer a given image
        with self.graph.as_default():
            input_image = np.expand_dims(image, axis = 0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.session.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict = {self.image_tensor: input_image}
            )
            end = datetime.datetime.now()
            total_time = end - start
            print("Inference Time: " + str(total_time.total_seconds()))

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        # Interpret score to generate prediction
        print('SCORES: ' + str(scores[0]))
        print('CLASSES: ' + str(classes[0]))

        if scores[0] > self.threshold:
            if classes[0] == 1:
                print('GREEN LIGHT')
                return TrafficLight.GREEN
            elif classes[0] == 2:
                print('RED LIGHT')
                return TrafficLight.RED
            elif classes[0] == 3:
                print('YELLOW LIGHT')
                return TrafficLight.YELLOW
                
        return TrafficLight.UNKNOWN
