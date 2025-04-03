#!/usr/bin/env python3
#edge_detection_client.py
import rospy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from edge_detection.srv import EdgeDetection, EdgeDetectionRequest

class EdgeDetectionClient:
    def __init__(self):
        rospy.init_node("edge_detection_client_node", anonymous=True)

        self.color_topic = rospy.get_param('~color_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw')
        self.service_name = rospy.get_param('~service_name', 'edge_detection_service')

        rospy.loginfo("Waiting for service: %s", self.service_name)
        rospy.wait_for_service(self.service_name)
        self.service = rospy.ServiceProxy(self.service_name, EdgeDetection)
        rospy.loginfo("Edge Detection Service is ready.")

        self.overlay_pub = rospy.Publisher("edge_points/overlay", Image, queue_size=1)
        self.marker_pub  = rospy.Publisher("edge_points/marker", Marker, queue_size=1)

        # Store the latest depth image
        self.latest_depth = None

        # Subscribe to the color and depth topics independently
        self.color_sub = rospy.Subscriber(self.color_topic, Image, self.color_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)

        # Define a time tolerance (seconds) for pairing color and depth messages
        self.tolerance = 0.1 

    def depth_callback(self, depth_msg):
        # Update the latest depth image received
        self.latest_depth = depth_msg

    def color_callback(self, color_msg):
        # Create a service request using the color image
        req = EdgeDetectionRequest()
        req.color = color_msg

        # If a depth image has been received, check if it is recent enough
        if self.latest_depth is not None:
            # Calculate the time difference between color and depth messages
            dt = abs((color_msg.header.stamp - self.latest_depth.header.stamp).to_sec())
            if dt < self.tolerance:
                req.depth = self.latest_depth


        try:
            resp = self.service(req)
            # Publish overlay if available
            if resp.overlay is not None:
                self.overlay_pub.publish(resp.overlay)
            # Publish marker if available and not empty
            if len(resp.marker.points) > 0:
                self.marker_pub.publish(resp.marker)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


def main():
    EdgeDetectionClient()
    rospy.spin()

if __name__ == '__main__':
    main()
