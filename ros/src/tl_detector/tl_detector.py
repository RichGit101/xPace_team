#!/usr/bin/env python
import rospy
from scipy.spatial import KDTree
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np

STATE_COUNT_THRESHOLD = 3

# When receiving images, skip this many, before reading in a new one. Default is 0.
# This is used to reduce computation load when the camera is on.
SKIP_IMAGES = 3
assert type(SKIP_IMAGES) is int and SKIP_IMAGES >= 0


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        # Track how many images were received
        self.images_received = 0

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
       #sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.waypoints_2D = None
        self.waypoint_tree = None

        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2D:
            self.waypoints_2D = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2D)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        # Skip images to reduce computation load
        self.images_received += 1
        if self.images_received % (SKIP_IMAGES + 1) != 0:
            return

        self.has_image = True
        self.camera_image = msg
        height = self.camera_image.height
        width = self.camera_image.width
        channels = len(self.camera_image.data)/(height * width)
        self.np_camera_img = np.fromstring(self.camera_image.data, np.uint8)
        self.np_camera_img = self.np_camera_img.reshape((height, width, channels))
        boxes, classes, confs = self.light_classifier.inference_for_single_image(self.np_camera_img)
        state = None
        if boxes.size > 0:
          states = self.light_classifier.get_classification(classes)
          state = states[np.argmax(confs)]
          if confs.shape != (1,): # Workaround for IndexError from indexing into shape (1,) 
              self.light_classifier.visualize_result(self.np_camera_img, boxes, classes, confs)
          light_wp = self.find_next_intersection()

        cv2.imshow('image', cv2.cvtColor(self.np_camera_img, cv2.COLOR_RGB2BGR))
        cv2.waitKey(10)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if state != None:
          if self.state != state:
              self.state_count = 0
              self.state = state
          elif self.state_count >= STATE_COUNT_THRESHOLD:
              self.last_state = self.state
              light_wp = light_wp if state == TrafficLight.RED else -1
              self.last_wp = light_wp
              self.upcoming_red_light_pub.publish(Int32(light_wp))
          else:
              self.upcoming_red_light_pub.publish(Int32(self.last_wp))
          self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            x (float): x coordinate of the waypoint
            y (float): y coordinate of the waypoint

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # TODO: check if this does not cause the car to stop in the middle of the intersection - we're not checking
        # if the traffic light is behind us. -Linas

        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # For testing, just return the light state
        return light.state

        # if(not self.has_image):
        #    self.prev_light_loc = None
        #    return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        ##Get classification
        # return self.light_classifier.get_classification(cv_image)

    def find_next_intersection(self):
        """Finds closest visible traffic light stop line and determines its
            location

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light

        """
        closest_line = None  # closest traffic light line
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose:
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            # TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)

            for i, line in enumerate(stop_line_positions):
              # Get stop line waypoint index
              temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
              # Find closest stop line waypoint index
              d = temp_wp_idx - car_wp_idx
              if 0 < d < diff:
                  diff = d
                  closest_line = line
                  line_wp_idx = temp_wp_idx
            
        return line_wp_idx

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
