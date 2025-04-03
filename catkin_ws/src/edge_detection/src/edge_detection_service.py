#!/usr/bin/env python3
#edge_detection_service.py
import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from edge_detector import EdgeDetector

class EdgeDetectionService:
    def __init__(self):
        rospy.init_node("edge_detection_service_node")
        self.bridge = CvBridge()

        # Load parameters
        method = rospy.get_param('~method', 'canny')
        canny_low = rospy.get_param('~canny_low_threshold', 50)
        canny_high = rospy.get_param('~canny_high_threshold', 150)
        sobel_ksize = rospy.get_param('~sobel_ksize', 3)
        laplacian_ksize = rospy.get_param('~laplacian_ksize', 3)

        # Camera intrinsics will be set from CameraInfo
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/depth/camera_info')
        rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)

        self.depth_scale = 0.001  # 16UC1 → meters
        self.edge_detector = EdgeDetector(
            method=method,
            canny_low_threshold=canny_low,
            canny_high_threshold=canny_high,
            sobel_ksize=sobel_ksize,
            laplacian_ksize=laplacian_ksize
        )

        self.service = rospy.Service("edge_detection_service", EdgeDetection, self.handle_request)
        rospy.loginfo("Edge Detection Service ready.")


    def camera_info_callback(self, msg):
        # Set camera intrinsics only once
        if self.fx is None:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            rospy.loginfo("Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                          self.fx, self.fy, self.cx, self.cy)
            

    def handle_request(self, req):
        try:
            color_img = self.bridge.imgmsg_to_cv2(req.color, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge error converting color image: %s", e)
            return EdgeDetectionResponse()
        # Process the color image and overlay edges
        edges = self.edge_detector.detect_edges(color_img)
        overlay = np.zeros_like(color_img)
        overlay[edges > 0] = (0, 255, 0)

        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge error converting overlay: %s", e)
            return EdgeDetectionResponse()

        # Create marker only if depth image data is available.
        marker = Marker()  # default empty marker
        if req.depth.data:  # check if the depth message contains data
            try:
                depth_img = self.bridge.imgmsg_to_cv2(req.depth, '16UC1').astype(np.float32)
                marker = self.create_marker(edges, depth_img, req.depth.header)
            except CvBridgeError as e:
                rospy.logerr("CvBridge error converting depth image: %s", e)

        return EdgeDetectionResponse(overlay=overlay_msg, marker=marker)

    def create_marker(self, edges, depth_img, header):
        points = []
        h, w = edges.shape
        for v in range(h):
            for u in range(w):
                if edges[v, u] > 0:
                    d = depth_img[v, u]
                    if d == 0:
                        continue
                    z = d * self.depth_scale
                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy
                    points.append(Point(x=x, y=y, z=z))

        m = Marker()
        m.header = header
        m.ns = "edge_points"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.002
        m.scale.y = 0.002
        m.color.a = 1.0
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.points = points
        return m

def main():
    EdgeDetectionService()
    rospy.spin()

if __name__ == '__main__':
    main()
