from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2 as cv
import numpy as np
from utils import label_map_util
from utils import visualization_utils as vis_util
import os

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        # TODO: Modify variables below
        PATH_TO_PB_FILE = os.path.abspath(__file__ + '/../../TL_detectors/sim_model/frozen_inference_graph.pb')
        PATH_TO_LABELS = os.path.abspath(__file__ + '/../../TL_detectors/sim_model/label_map.pbtxt')
        self.NUM_CLASSES = 4
        self.THRESHOLD = 0.5

        self.graph = tf.Graph()

        # Detection model loaded
        with self.graph.as_default():
            graph_def = tf.GraphDef()

            # Parsing computational graph from pb file
            with tf.gfile.GFile(PATH_TO_PB_FILE, 'rb') as fid:
                serialized = fid.read()
                graph_def.ParseFromString(serialized)
                tf.import_graph_def(graph_def, name='')

            # Those are the placeholder for the detection
            self.input_img = self.graph.get_tensor_by_name('image_tensor:0')
            self.res_bbox = self.graph.get_tensor_by_name('detection_boxes:0')
            self.res_score = self.graph.get_tensor_by_name('detection_scores:0')
            self.res_classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detection = self.graph.get_tensor_by_name('num_detections:0')

        # Load pbtxt file into memory
        self.label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(
            self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=False
        )
        self.category_index = label_map_util.create_category_index(categories)

    def inference_for_single_image(self, image):
        """
        does the inference and returns the result.
        Args:
            image: 3D array where the shape is [height, width, 3]

        Returns:
            tuple<np.ndarray>: tuple of np.ndarrays where first element is the detected boxes,
                and the second element is detected classes, and the last one is the detected
                confidences. \
                positive_boxes is the 2D array where the shape is [N, 4] and for each
                element has the following information: [ymin, xmin, ymax, xmax]. Keep note they
                are in normalized format and the range is [0, 1]

                positive_classes is 2D array with the shape of [N, 1]. ith detected box's class
                can be found by positive_classes[i][0]

                positive_confidences is 2D array with the shape of [N, 1]. ith detected box's
                confidence can be found by positive_confidences[i][0] which is in range of [0, 1.0]

        """
        # Start session for a prediction
        with tf.Session(graph=self.graph) as sess:

            # Run the detector!!!
            (boxes, scores, classes, num) = sess.run(
                [self.res_bbox, self.res_score, self.res_classes, self.num_detection],
                feed_dict={self.input_img: image[None, ...]}
            )

            # Get detections with the confidence >= self.THRESHOLD
            positive_indices = np.where(scores[0] >= self.THRESHOLD)[0]
            positive_boxes = boxes[0][positive_indices]
            positive_classes = classes[0][positive_indices]
            positive_confidences = scores[0][positive_indices]

        return (positive_boxes, positive_classes, positive_confidences)

    def visualize_result(self, image, boxes, classes, confidences):
        """
        Visualize bounding boxes on the image in place
        Args:
            image: 3D array where the shape is [height, width, 3]
            boxes: 2D array which is returned by inference_for_single_image method
            classes: 2D array which is returned by inference_for_single_image method
            confidences: 2D array which is returned by inference_for_single_image method

        Returns:

        """
        vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(confidences),
            self.category_index,
            use_normalized_coordinates=True,
            min_score_thresh=0.5,
            line_thickness=3)

    def get_classification(self, classes):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        states = []
        for i, a_class in enumerate(classes):
          class_name = self.category_index[classes[i]]['name']
          if class_name == 'Green':
            states.append(TrafficLight.GREEN)
          elif class_name == 'Red':
            states.append(TrafficLight.RED)
          elif class_name == 'Yellow':
            states.append(TrafficLight.YELLOW)
          elif class_name == 'off':
            states.append(TrafficLight.OFF)
          else:
            states.append(TrafficLight.UNKNOWN)

        return states
