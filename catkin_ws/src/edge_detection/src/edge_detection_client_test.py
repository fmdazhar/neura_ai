#!/usr/bin/env python3
#edge_detection_client_test.py
import os
import glob
import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from edge_detection.srv import EdgeDetection, EdgeDetectionRequest

def main():
    # Initialize this node
    rospy.init_node('edge_detection_batch_client', anonymous=True)

    # Parameters
    input_dir = rospy.get_param('~input_dir', '/root/catkin_ws/src/edge_detection/data/images')
    output_dir = rospy.get_param('~output_dir', '/root/catkin_ws/results/service_client/python')
    service_name = rospy.get_param('~service_name', 'edge_detection_service')

    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Create a CvBridge for conversions
    bridge = CvBridge()

    # Wait for the EdgeDetection service to be available
    rospy.loginfo(f"Waiting for service [{service_name}] ...")
    rospy.wait_for_service(service_name)
    rospy.loginfo("Service is now available.")
    detect_edges = rospy.ServiceProxy(service_name, EdgeDetection)

    # Gather images from input directory
    extensions = ('*.png', '*.jpg', '*.jpeg', '*.bmp', '*.tiff')
    image_files = []
    for ext in extensions:
        image_files.extend(glob.glob(os.path.join(input_dir, ext)))

    if not image_files:
        rospy.logwarn(f"No images found in {input_dir}. Exiting.")
        return

    for img_path in image_files:
        basename = os.path.splitext(os.path.basename(img_path))[0]
        # Load the image as BGR
        bgr = cv2.imread(img_path)
        if bgr is None:
            rospy.logwarn(f"Failed to load {img_path}, skipping.")
            continue

        # Convert to ROS Image
        try:
            ros_image = bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error converting {img_path}: {e}")
            continue

        # Create service request
        req = EdgeDetectionRequest()
        req.color = ros_image
        # No depth – we'll leave req.depth as the default empty sensor_msgs/Image

        try:
            resp = detect_edges(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed for {img_path}: {e}")
            continue

        # Convert response overlay to cv2 and save
        if resp.overlay.data:
            try:
                overlay_cv = bridge.imgmsg_to_cv2(resp.overlay, 'bgr8')
                outpath = os.path.join(output_dir, f"{basename}_edges.png")
                cv2.imwrite(outpath, overlay_cv)
                rospy.loginfo(f"Saved edges for {img_path} to {outpath}")
            except CvBridgeError as e:
                rospy.logerr(f"Error converting overlay for {img_path}: {e}")
        else:
            rospy.logwarn(f"No overlay data received for {img_path}.")

    rospy.loginfo("Batch edge detection complete.")

if __name__ == '__main__':
    main()
