// EdgeDetectorNode.cpp
#include "edge_detection/EdgeDetectorNode.hpp"

namespace edge_detection
{

EdgeDetectorNode::EdgeDetectorNode() 
  : nh_("~"),
    fx_(0.0f),
    fy_(0.0f),
    cx_(0.0f),
    cy_(0.0f),
    camera_info_received_(false),
    has_color_and_depth_(false),
    canny_low_threshold_(50),
    canny_high_threshold_(150),
    edge_detector_(50, 150)  // initialize the edge detector with default Canny thresholds
{
  // Read ROS params (private nodehandle "~")
  nh_.param("canny_low_threshold",   canny_low_threshold_,   50);
  nh_.param("canny_high_threshold",  canny_high_threshold_, 150);

  // Re-initialize EdgeDetector with updated threshold params
  edge_detector_ = edge_detection::EdgeDetector(canny_low_threshold_, canny_high_threshold_);

  // Topics
  std::string image_topic;
  std::string depth_topic;
  std::string camera_info_topic;
  nh_.param<std::string>("image_topic",       image_topic,       "/camera/color/image_raw");
  nh_.param<std::string>("depth_topic",       depth_topic,       "/camera/depth/image_rect_raw");
  nh_.param<std::string>("camera_info_topic", camera_info_topic, "/camera/depth/camera_info");

  // Set up subscribers
  color_sub_.subscribe(nh_, image_topic, 1);
  depth_sub_.subscribe(nh_, depth_topic, 1);


  // Approximate Time Synchronizer
  sync_ = std::make_shared<Sync>(SyncPolicy(10), color_sub_, depth_sub_);
  sync_->setMaxIntervalDuration(ros::Duration(0.01));
  sync_->registerCallback(boost::bind(&EdgeDetectorNode::syncedCallback, this, _1, _2));

  // Camera info subscriber
  info_sub_ = nh_.subscribe(camera_info_topic, 1,
                            &EdgeDetectorNode::cameraInfoCallback, this);

  // Publishers
  edge_image_pub_  = nh_.advertise<sensor_msgs::Image>("/edge_points/overlay", 1);
  edge_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/edge_points/marker", 1);
  
  ROS_INFO("EdgeDetectorNode started. Waiting for images...");
}

void EdgeDetectorNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  if (!camera_info_received_)
  {
    fx_ = static_cast<float>(msg->K[0]);
    fy_ = static_cast<float>(msg->K[4]);
    cx_ = static_cast<float>(msg->K[2]);
    cy_ = static_cast<float>(msg->K[5]);

    camera_info_received_ = true;
    ROS_INFO("Camera intrinsics received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
             fx_, fy_, cx_, cy_);
  }
}

void EdgeDetectorNode::syncedCallback(const sensor_msgs::ImageConstPtr &color_msg,
  const sensor_msgs::ImageConstPtr &depth_msg)
{
  try
  {
  cv_bridge::CvImageConstPtr color_cv_ptr = cv_bridge::toCvShare(color_msg, "bgr8");
  latest_color_ = color_cv_ptr->image.clone();

  cv_bridge::CvImageConstPtr depth_cv_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv::Mat depth_16UC1 = depth_cv_ptr->image;
  depth_16UC1.convertTo(latest_depth_, CV_32FC1); // convert to float

  depth_frame_id_   = depth_msg->header.frame_id;
  depth_msg_stamp_  = depth_msg->header.stamp;
  has_color_and_depth_ = true;
  }
  catch (cv_bridge::Exception &e)
  {
  ROS_ERROR("cv_bridge exception: %s", e.what());
  return;
  }

  processIfReady();
}



void EdgeDetectorNode::processIfReady()
{ 
  if (!has_color_and_depth_)
  {
    ROS_WARN("Waiting for color and depth images...");
    return;
  }
  // Detect edges (Canny only for now)
  cv::Mat edges = edge_detector_.detectEdges(latest_color_);

  // Create overlay image (highlight edges in green)
  cv::Mat overlay = cv::Mat::zeros(latest_color_.size(), latest_color_.type());
  for (int v = 0; v < edges.rows; ++v)
  {
    for (int u = 0; u < edges.cols; ++u)
    {
      if (edges.at<uchar>(v, u) > 0)
      {
        // Set the pixel to green (BGR)
        overlay.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 255, 0);
      }
    }
  }

  // Publish the overlay image
  sensor_msgs::ImagePtr overlay_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", overlay).toImageMsg();
  edge_image_pub_.publish(overlay_msg);

  // Create and publish Marker for 3D edge points only if camera intrinsics are available
  if (camera_info_received_)
  {
    visualization_msgs::Marker marker = createMarkerForEdges(edges, latest_depth_);
    if (!marker.points.empty())
    {
      edge_marker_pub_.publish(marker);
    }
    else
    {
      ROS_WARN("No edge points found for marker.");
    }
  }
  else
  {
    ROS_WARN("Camera info not received, skipping marker creation.");
  }
}


visualization_msgs::Marker EdgeDetectorNode::createMarkerForEdges(const cv::Mat &edges, const cv::Mat &depth_img)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = depth_frame_id_;
  marker.header.stamp    = depth_msg_stamp_;
  marker.ns = "edge_points";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  // Size + color
  marker.scale.x = 0.002;
  marker.scale.y = 0.002;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // Depth scale factor (16UC1 -> meters). 
  // If your camera driver already accounts for scale, you might not need this.
  const float depth_scale = 0.001f;  

  for (int v = 0; v < edges.rows; ++v)
  {
    for (int u = 0; u < edges.cols; ++u)
    {
      if (edges.at<uchar>(v, u) > 0)
      {
        float d = depth_img.at<float>(v, u);
        // If d == 0 or invalid, skip
        if (d <= 0.0f)
          continue;

        float Z = d * depth_scale;
        float X = (u - cx_) * Z / fx_;
        float Y = (v - cy_) * Z / fy_;

        geometry_msgs::Point pt;
        pt.x = X;
        pt.y = Y;
        pt.z = Z;
        marker.points.push_back(pt);
      }
    }
  }

  if (marker.points.empty())
  {
    ROS_WARN("No edge points found for Marker.");
  }
  return marker;
}

} // namespace edge_detection


/*******************************
 *           main()
 *******************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "edge_detector_node");
  edge_detection::EdgeDetectorNode node;
  ros::spin();
  return 0;
}
