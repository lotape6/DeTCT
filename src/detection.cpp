#include <ros/ros.h>
#include <ros/console.h>
#include "detection.h"
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

namespace enc = sensor_msgs::image_encodings;


detection::detection(ros::NodeHandle nh_, std::string image_topic,std::string depth_topic,std::string pose_topic)
  : it_(nh_),
  // Subscribe to bgr img, depth img and to satamped pose
  image_sub_(nh_,image_topic, 1),
  depth_sub_(nh_,depth_topic, 1),
  // Initialize Approx time synchronizer
  sync(MySyncPolicy(10),image_sub_, depth_sub_)
{
  pose_sub_ = nh_.subscribe(pose_topic, 1, &detection::poseCB, this);

  // Initialize the publisher and assing callback to the synchronizer
  getInputParams(nh_);
  image_pub_ = it_.advertise("/image_converter/output_video", 5);
  sync.registerCallback(boost::bind(&detection::imageCb,this, _1, _2));
  lastest_marker_id = 0;
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/detct/estimated_objects_markers", 1);
  sync_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("detct/sync_poseStamped",1);
}

detection::~detection()
{
}


void detection::getInputParams(ros::NodeHandle nh_){
  // std::string foo;
  if( nh_.getParam("/detection/image_proc_params/h_low", H_LOWER_THRESHOLD) &&
      nh_.getParam("/detection/image_proc_params/h_high",H_UPPER_THRESHOLD) &&
      nh_.getParam("/detection/image_proc_params/s_low", S_LOWER_THRESHOLD) &&
      nh_.getParam("/detection/image_proc_params/s_high",S_UPPER_THRESHOLD) &&
      nh_.getParam("/detection/image_proc_params/v_low", V_LOWER_THRESHOLD) &&
      nh_.getParam("/detection/image_proc_params/v_high",V_UPPER_THRESHOLD) &&
      nh_.getParam("/detection/image_proc_params/morph_op_erosion_size",EROSION_SIZE) &&
      nh_.getParam("/detection/image_proc_params/morph_op_dilation_size",DILATION_SIZE) &&
      nh_.getParam("/detection/image_proc_params/min_obj_width",MIN_OBJ_WIDTH) &&
      nh_.getParam("/detection/image_proc_params/min_obj_height",MIN_OBJ_HEIGHT) &&
      nh_.getParam("/detection/image_proc_params/depth_segmentation_tolerance",DEPTH_THRESHOLD_TOLERANCE) &&
      nh_.getParam("/detection/image_proc_params/depth_bound_expansion_coef",DEPTH_BOUND_RECT_EXPANSION_COEF) &&
      nh_.getParam("/detection/image_proc_params/use_depth_for_detection",USE_DEPTH_FOR_DETECTION)){

    ROS_INFO_STREAM("Image processing parameters received");
     }
  else{
    ROS_ERROR("Image processing parameters not received!");
  }

}

//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//

// Define the image [bgr, depth] callback (which also takes position)
void detection::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
             const sensor_msgs::ImageConstPtr& depth_msg)  {

  //OpenCV_Bridge
  cv_bridge::CvImage img_bridge;

  //ROS image image_msg
  sensor_msgs::Image image_out_msg; // >> message to be sent
  cv_bridge::CvImagePtr bgr_ptr, depth_ptr;

  //Euler angles and position of the camera
  double roll, pitch, yaw;

  //OpenCV images
  cv::Mat edges =           cv::Mat::zeros(image_msg->height, image_msg->width, CV_8UC1);
  cv::Mat b_mask =          cv::Mat::zeros(image_msg->height, image_msg->width, CV_8UC3);
  cv::Mat b_mask_combined = cv::Mat::zeros(image_msg->height, image_msg->width, CV_8UC1);
  cv::Mat b_mask_float =    cv::Mat::zeros(image_msg->height, image_msg->width, CV_32FC1);
  cv::Mat drw =             cv::Mat::zeros(image_msg->height, image_msg->width, CV_8UC3);
  cv::Mat dist=             cv::Mat::zeros(image_msg->height, image_msg->width, CV_32FC1);
  cv::Mat hsv_img;

  //Image iterators
  int icont, jcont;
  float actual_float;



  //Contours variables
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  //---------------------End of variable declarations----------------------//


  std_msgs::Header h = image_msg->header;

  geometry_msgs::PoseStamped p = last_vicon_pose;
  tf2::Quaternion camera_pose_quat(p.pose.orientation.x,
                                   p.pose.orientation.y,
                                   p.pose.orientation.z,
                                   p.pose.orientation.w);

  //Create transformation between global frame and camera frame
  // World to Camera Transformation = W_C_Transform
  tf2::Vector3 CameraTranslation(p.pose.position.x,
                                 p.pose.position.y,
                                 p.pose.position.z);

  tf2::Transform W_C_Transform(camera_pose_quat,CameraTranslation);

  tf2::Matrix3x3(camera_pose_quat).getRPY(roll, pitch, yaw);


  try {
    bgr_ptr = cv_bridge::toCvCopy(image_msg, enc::BGR8);
    depth_ptr = cv_bridge::toCvCopy(depth_msg, "32FC1");
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::cvtColor(bgr_ptr->image, hsv_img, cv::COLOR_BGR2HSV);

  //Perform a lower and higher threshold and perform a closing (erosion + dilation)
  b_mask = imageProcessing(hsv_img);

  if(USE_DEPTH_FOR_DETECTION){
    improveWithDepth (b_mask_combined, b_mask, depth_ptr->image);
  }
  else{
    b_mask_combined=b_mask;
  }

  // Multiply the binary mask and the depth image to get approximate distance
  b_mask_combined.convertTo(b_mask_float, CV_32FC1, 1/255.0);
  dist=depth_ptr->image.mul(b_mask_float);

  cv::findContours( b_mask_combined, contours, hierarchy,
                    cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );

  /// Approximate contours to polygons + get bounding
  std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
  std::vector<cv::Rect> boundRect( contours.size() );
  std::vector<cv::Rect> aerealBoundRect( contours.size() );
  std::vector<float> objDist( contours.size(), 0.0 );
  std::vector<geometry_msgs::Point> objPoseEst(contours.size());

  int n_objects = 0;
  bool obj_is_big_enough;

  for( int i = 0; i < contours.size(); i++ ){   //Iterating through contours

    //Defined precision polygon approximation
    cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
    boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
    cv::Scalar color( i*25, (contours.size()-i)*25, i*25 );

    obj_is_big_enough=true;

    //Discard small objects
    if (boundRect[i].height < MIN_OBJ_HEIGHT && boundRect[i].width < MIN_OBJ_WIDTH){
      obj_is_big_enough = false;
    }

    //Get a small rectangle centered inside the boundRect to avoid distance errors
    int dist_values;
    int x_init=boundRect[i].tl().x+boundRect[i].width/4;
    int y_init=boundRect[i].tl().y+boundRect[i].height/4;
    int x_fin=boundRect[i].br().x-boundRect[i].width/4;
    int y_fin=boundRect[i].br().y-boundRect[i].height/4;

    if (obj_is_big_enough){

      objDist[i]=getObjDistance(dist,x_init,x_fin,y_init,y_fin);
      tf2::Vector3 ObjectPosition;
      ObjectPosition=getObjEstPose(W_C_Transform,boundRect[i], objDist[i],x_init,y_init);
      objPoseEst[n_objects].x=ObjectPosition.getX();
      objPoseEst[n_objects].y=ObjectPosition.getY();
      objPoseEst[n_objects].z=ObjectPosition.getZ();
      n_objects ++;

      //Drawing contours
      cv::drawContours( drw, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
      cv::rectangle( drw, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
    }
 }// End of contours iteration

  publishMarkers(n_objects, objPoseEst);

  img_bridge = cv_bridge::CvImage(h, "8UC3" ,drw);
  img_bridge.toImageMsg(image_out_msg);
  image_pub_.publish(image_out_msg);
  sync_pose_pub_.publish(p);
}

//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//

void detection::poseCB(const geometry_msgs::PoseStampedConstPtr& msg){
  last_vicon_pose = *msg;
}

//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//

cv::Mat detection::imageProcessing(cv::Mat hsv_img){
  cv::Mat b_mask, element;
  cv::inRange(hsv_img,
              cv::Scalar(H_LOWER_THRESHOLD, S_LOWER_THRESHOLD, V_LOWER_THRESHOLD),
              cv::Scalar(H_UPPER_THRESHOLD, S_UPPER_THRESHOLD, V_UPPER_THRESHOLD),
              b_mask);

  // Define Erosion operation
  element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 2*EROSION_SIZE + 1, 2*EROSION_SIZE+1 ),
                                       cv::Point( EROSION_SIZE, EROSION_SIZE ) );
  // Apply the erosion operation
  cv::erode( b_mask, b_mask, element );

  element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 2*DILATION_SIZE + 1, 2*DILATION_SIZE+1 ),
                                       cv::Point( DILATION_SIZE, DILATION_SIZE ) );
  cv::dilate( b_mask, b_mask, element );

  return b_mask;
}

//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//

void detection::improveWithDepth (cv::Mat &b_mask_combined, cv::Mat &b_mask, cv::Mat &depth ){

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat element;
  cv::Mat b_mask_aux = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1);

  cv::findContours( b_mask, contours, hierarchy,
                    cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );
  std::vector<std::vector<cv::Point> > contours_poly_init( contours.size() );
  std::vector<cv::Rect> boundRectInit( contours.size() );
  float actual_float;
  for (int i=0; i < contours.size(); i++ ){
    cv::approxPolyDP( cv::Mat(contours[i]), contours_poly_init[i], 3, true );
    boundRectInit[i] = cv::boundingRect( cv::Mat(contours_poly_init[i]) );

    //Define bounding limits
    int x_init=boundRectInit[i].tl().x;
    int y_init=boundRectInit[i].tl().y;
    int x_fin=boundRectInit[i].br().x;
    int y_fin=boundRectInit[i].br().y;
    bool depth_founded = false;     //For exiting flag
    //Search for a depth value corresponding with a point of the b_mask
    for( int icont = x_init; icont < x_fin; icont++ ){
      for( int jcont = y_init; jcont < y_fin; jcont++ ){
        if(b_mask.at<int>(jcont,icont)) {
          actual_float = depth.at<float>(jcont, icont);
          if(actual_float>0 && actual_float<100) {depth_founded=true; break;}
        }
      }
      if(depth_founded) break;
    }

    //Get the binary mask of the objects at a given distance from depth image
    cv::inRange(depth,
                actual_float-DEPTH_THRESHOLD_TOLERANCE,
                actual_float+DEPTH_THRESHOLD_TOLERANCE,
                b_mask_aux);

    //Create mask to eliminate objects out of the corresponding bounding box
    cv::Mat mask = cv::Mat::zeros( depth.rows, depth.cols, CV_8UC1); // all 0

    //If the bounding rect is in the limit do not expand the bounds
    if( x_init-DEPTH_BOUND_RECT_EXPANSION_COEF < 0 ||
        y_init-DEPTH_BOUND_RECT_EXPANSION_COEF < 0 ||
        x_fin+DEPTH_BOUND_RECT_EXPANSION_COEF > 2*V_CENTER ||
        y_fin+DEPTH_BOUND_RECT_EXPANSION_COEF >2*H_CENTER){
      cv::rectangle(mask, cv::Point(x_init, y_init),
                          cv::Point(x_fin, y_fin),
                          cv::Scalar(1), -1); //-1 = FILLED
    }
    //Expand the bounding limits to get the full object
    else{
      cv::rectangle(mask, cv::Point(x_init-DEPTH_BOUND_RECT_EXPANSION_COEF,
                                    y_init-DEPTH_BOUND_RECT_EXPANSION_COEF),
                          cv::Point(x_fin+DEPTH_BOUND_RECT_EXPANSION_COEF,
                                    y_fin+DEPTH_BOUND_RECT_EXPANSION_COEF),
                          cv::Scalar(1), -1); //-1 = FILLED
    }

    b_mask_aux = b_mask_aux.mul(mask);

    element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                         cv::Size( 2*EROSION_SIZE + 1, 2*EROSION_SIZE+1 ),
                                         cv::Point( EROSION_SIZE, EROSION_SIZE ) );
    // Apply the erosion operation
    cv::erode( b_mask_aux, b_mask_aux, element );

    element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                         cv::Size( 2*DILATION_SIZE + 1, 2*DILATION_SIZE+1 ),
                                         cv::Point( DILATION_SIZE, DILATION_SIZE ) );
    cv::dilate( b_mask_aux, b_mask_aux, element );

    b_mask_combined = b_mask_combined + b_mask_aux + b_mask ;
  }
}

//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//

float detection::getObjDistance(cv::Mat depth_img, int x_init, int x_fin, int y_init, int y_fin){

  float actual_float=0.0;
  float object_distance = 0.0;
  int dist_values=0;

  for( int i = x_init; i < x_fin; i++ ){
    for( int j = y_init; j < y_fin; j++ ){

      //Get the distance value
      actual_float = depth_img.at<float>(j, i);

      //Chech for valid distance value
      if ( actual_float > 0.0 && actual_float < 100.0){
        object_distance = object_distance + actual_float;
        dist_values ++;
      }
    }
  }
  // Get the average distance of the object i
  return (object_distance/dist_values);
}

//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//

tf2::Vector3 detection::getObjEstPose(tf2::Transform W_C_Transform, cv::Rect boundRect,float objDist,int x_init,int y_init) {

  //Calculate camera_relative angle of the object i
  double rot_z=(x_init+(boundRect.width/2.0)-H_CENTER)*HAP;
  double rot_y=(y_init+(boundRect.height/2.0)-V_CENTER)*VAP;
  tf2::Vector3 ObjectTranslation(objDist,0,0);
  tf2::Quaternion ObjectRotation;
  ObjectRotation.setRPY((double) 0,rot_y,rot_z);
  tf2::Transform C_I_Transform(ObjectRotation,ObjectTranslation);
  C_I_Transform.mult(W_C_Transform,C_I_Transform);
  return ObjectTranslation=C_I_Transform.getOrigin();
}

//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//

void detection::publishMarkers(int n_markers, std::vector<geometry_msgs::Point> objects_poses){

  visualization_msgs::MarkerArray markersArray;

  for (int i = 0; i < n_markers; i++){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "vicon";
    marker.header.stamp = ros::Time();
    marker.ns = "detct";
    marker.id = lastest_marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = objects_poses[i].x;
    marker.pose.position.y = objects_poses[i].y;
    marker.pose.position.z = objects_poses[i].z;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.g = 1.0;
    lastest_marker_id ++;
    markersArray.markers.push_back(marker);
  }
  markers_pub_.publish( markersArray );

}

//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect");
  ros::NodeHandle nh_;
  std::string image_topic, depth_topic, pose_topic;
  if(nh_.getParam("/detection/config/image_topic", image_topic) &&
     nh_.getParam("/detection/config/depth_topic", depth_topic) &&
     nh_.getParam("/detection/config/pose_topic", pose_topic)){

    ROS_INFO_STREAM("Topic names received");
     }
  else{
    ROS_ERROR("Parameters not received!");
  }
  detection ic(nh_, image_topic, depth_topic, pose_topic);
  while(ros::ok()){
    ros::spin();
  }
  return 0;
}
