//EdgeDetectorNode.hpp
#ifndef EDGE_DETECTION_EDGE_DETECTOR_NODE_HPP_
#define EDGE_DETECTION_EDGE_DETECTOR_NODE_HPP_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Include your EdgeDetector
#include "edge_detection/EdgeDetector.hpp"

namespace edge_detection
{

class EdgeDetectorNode
{
public:
  EdgeDetectorNode();
  ~EdgeDetectorNode() = default;

private:
  // ROS-related
  ros::NodeHandle nh_;
  ros::Subscriber info_sub_;
  ros::Subscriber color_only_sub_; 
  ros::Publisher  edge_image_pub_;
  ros::Publisher  edge_marker_pub_;

  // Message filter subscribers
  message_filters::Subscriber<sensor_msgs::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;

  // Approximate time sync
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  // Camera intrinsics
  float fx_;
  float fy_;
  float cx_;
  float cy_;
  bool camera_info_received_;

  // Buffers
  cv::Mat latest_color_;
  cv::Mat latest_depth_;
  std::string depth_frame_id_;
  ros::Time depth_msg_stamp_;
  bool has_color_and_depth_;

  // Edge detector
  edge_detection::EdgeDetector edge_detector_; 

  // ROS param
  int canny_low_threshold_;
  int canny_high_threshold_;

  // Callbacks
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  void syncedCallback(const sensor_msgs::ImageConstPtr &color_msg,
                      const sensor_msgs::ImageConstPtr &depth_msg);
  void colorCallback(const sensor_msgs::ImageConstPtr &color_msg); 

  // Processing
  void processIfReady();
  visualization_msgs::Marker createMarkerForEdges(const cv::Mat &edges, const cv::Mat &depth_img);

};

} // namespace edge_detection

#endif  // EDGE_DETECTION_EDGE_DETECTOR_NODE_HPP_
