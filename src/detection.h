#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>//
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>
#include <tf/tf.h>///transform_datatypes.h
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/cache.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#ifndef DETECTION_H
#define DETECTOION_H


///////////////////////////////////////////////////////////////////////////
//                    IMAGE PROCESSING PARAMETERS                        //
///////////////////////////////////////////////////////////////////////////

// // Define HSV limits for color segmentation
// #define LOW_H 7
// #define HIGH_H 45     //ORANGE
// #define LOW_S 110
// #define HIGH_S 255    //ELSE WHITE OR GRAY
// #define LOW_V 64
// #define HIGH_V 255    //ELSE BLACK
//
//
// // Object size discrimination
// #define MIN_OBJ_HEIGH 20      //px
// #define MIN_OBJ_WIDTH 20      //px
//
// // Object depth tolerance for inRange
// #define DEPTH_TOLERANCE 0.1  //m
//
// // Bounding expansion due to bad color segmentation
// #define EXP_COEF 10          //px (Expansion Coeficient)



////////////////////////////////////////////////////////////////////////////
//                Angle per pixel of the camera                           //
////////////////////////////////////////////////////////////////////////////
//  Horizontal Field Of View = 58 (deg)  1.012291  (rad)                  //
//  Vertical Field Of View   = 45 (deg)  0.7853982 (rad)                  //
//  Resolution = 640x480 (px)                                             //
//                                                                        //
//  Horiz. Angle per Pixel(HAP) = 1.012291/640 = 0.00158170468  (rad/px)  //
//  Vert. Angle per Pixel (VAP) = 0.7853982/480 = 0.00163624625 (rad/px)  //
////////////////////////////////////////////////////////////////////////////
// *******! Please note that if any camera parameter changes,  !*******
// *******!      you might changes the values below            !*******
#define HAP 0.00158170468   //(rad/px)
#define VAP 0.00163624625   //(rad/px)
#define H_CENTER 320        //px
#define V_CENTER 240        //px


class detection
{
public:
  detection(ros::NodeHandle nh_, std::string image_topic,std::string depth_topic,std::string pose_topic);
  ~detection();

  void publishMarkers(int n_markers, std::vector<geometry_msgs::Point> objects_poses);
  void imageCb( const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::ImageConstPtr& depth_msg);
  void poseCB(const geometry_msgs::PoseStampedConstPtr& msg);
  cv::Mat imageProcessing(cv::Mat hsv_img);
  void getInputParams(ros::NodeHandle nh_);
  void improveWithDepth (cv::Mat &b_mask_combined, cv::Mat &b_mask, cv::Mat &depth );
  tf2::Vector3 getObjEstPose(tf2::Transform W_C_Transform, cv::Rect boundRect, float objDist,int x_init,int y_init);
  float getObjDistance(cv::Mat depth_img, int x_init, int x_fin, int y_init, int y_fin);
private:

  image_transport::ImageTransport it_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;

  ros::Subscriber pose_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image
  > MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync;


  image_transport::Publisher image_pub_;
  ros::Publisher sync_pose_pub_;
  ros::Publisher markers_pub_;

  int lastest_marker_id;

  geometry_msgs::PoseStamped last_vicon_pose;

  int H_UPPER_THRESHOLD, H_LOWER_THRESHOLD,
      S_UPPER_THRESHOLD, S_LOWER_THRESHOLD,
      V_UPPER_THRESHOLD, V_LOWER_THRESHOLD,
      EROSION_SIZE, DILATION_SIZE,
      MIN_OBJ_HEIGHT, MIN_OBJ_WIDTH,
      DEPTH_BOUND_RECT_EXPANSION_COEF;

  float DEPTH_THRESHOLD_TOLERANCE,
        POSE2CAMERA_DELAY;

  bool USE_DEPTH_FOR_DETECTION;

};




#endif //DETECTION_H
