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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/cache.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
///////////////////////////////////////////////////////////////////////////
//                    IMAGE PROCESSING PARAMETERS                        //
///////////////////////////////////////////////////////////////////////////

// Define HSV limits for color segmentation
#define LOW_H 7
#define HIGH_H 45     //ORANGE
#define LOW_S 110
#define HIGH_S 255    //ELSE WHITE OR GRAY
#define LOW_V 64
#define HIGH_V 255    //ELSE BLACK


// Object size discrimination
#define MIN_OBJ_HEIGH 20      //px
#define MIN_OBJ_WIDTH 20      //px

// Object depth tolerance for inRange
#define DEPTH_TOLERANCE 0.1  //m

// Bounding expansion due to bad color segmentation
#define EXP_COEF 10          //px (Expansion Coeficient)

// Edges detection parameters
#define EDGE_THRESHOLD 40    // THRESHOLD_INF = EDGE_THRESHOLD
#define EDGE_RATIO 8        // THRESHOLD_SUP = EDGE_THRESHOLD+EDGE_RATIO
#define EDGE_OPENING_SIZE 3

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

namespace enc = sensor_msgs::image_encodings;

class detection
{
public:
  detection()
    : it_(nh_),
    // Subscribe to bgr img, depth img and to satamped pose
    image_sub_(nh_,"/image", 1),
    depth_sub_(nh_,"/camera/depth/image", 1),
    pose_sub_(nh_,"/pose",1),
    // Initialize Approx time synchronizer and pose cache
    sync(MySyncPolicy(10),image_sub_, depth_sub_),
    cache(pose_sub_, 100)
    // Pose is cached due to his bigger rate
{
    // Initialize the publisher and assing callback to the synchronizer
    image_pub_ = it_.advertise("/image_converter/output_video", 5);
    sync.registerCallback(boost::bind(&detection::imageCb,this, _1, _2));
    lastest_marker_id = 0;
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/detct/estimated_objects_markers", 1);
  }

  ~detection()
  {
  }
  void publishMarkers(int n_markers, std::vector<geometry_msgs::Point> objects_poses){

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
        marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;


      lastest_marker_id ++;
      markersArray.markers.push_back(marker);
    }
    markers_pub_.publish( markersArray );

  }

  // Define the image [bgr, depth] callback (which also takes position)
  void imageCb(const sensor_msgs::ImageConstPtr& msg,
               const sensor_msgs::ImageConstPtr& depth_msg)  {
    //-----------------------Define variables-----------------------//
    //CV_Bridge
    cv_bridge::CvImage img_bridge;

    //ROS image msg
    sensor_msgs::Image img_msg; // >> message to be sent
    cv_bridge::CvImagePtr bgr_ptr, depth_ptr;

    //Euler angles and position of the camera
    double roll, pitch, yaw;
    float x, y, z;

    //OpenCV images
    cv::Mat edges = cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
    cv::Mat b_mask = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);
    cv::Mat b_mask_aux = cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
    cv::Mat b_mask_combined = cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
    cv::Mat b_mask_float = cv::Mat::zeros(msg->height, msg->width, CV_32FC1);
    cv::Mat drw = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);
    cv::Mat dist= cv::Mat::zeros(msg->height, msg->width, CV_32FC1);
    cv::Mat hsv_img, bgr[3];

    //Image iterators
    int icont, jcont;
    float actual_float;

    //Morphological operations (er_size = erosion size)
    int er_size;
    cv::Mat element;

    //Contours variables
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    //-----------------End of variable declarations------------------//

    //Get image header
    std_msgs::Header h = msg->header;

    //---------------------------Get Pose----------------------------//
    //Get the a pose close to image timestamp
    geometry_msgs::PoseStampedConstPtr p;
    p=cache.getElemAfterTime(h.stamp);

    tf::Quaternion quat(p->pose.orientation.x,
                        p->pose.orientation.y,
                        p->pose.orientation.z,
                        p->pose.orientation.w);

    //Get the position of the camera
    x=p->pose.position.x;
    y=p->pose.position.y;
    z=p->pose.position.z;

    //Create transformation between global frame and camera frame
    // World to Camera Transformation = W_C_Transform
    tf::Vector3 CameraTranslation(x,y,z);
    tf::Transform W_C_Transform(quat,CameraTranslation);


    //Transform quaternions to Euler angles
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    //DEBUG MESSAGES
    // ROS_INFO_STREAM("X: "<<p->pose.position.x<<" , Y: "<<p->pose.position.y<<" , Z: "<<p->pose.position.z);
    // ROS_INFO_STREAM("Roll: "<<roll<<" , Pitch: "<<pitch<<" , Yaw: "<<yaw);



    //--------------------------Get images---------------------------//
    try {
      bgr_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      depth_ptr = cv_bridge::toCvCopy(depth_msg, "32FC1");
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //Split bgr image into separated single channel image
    cv::split(bgr_ptr->image,bgr);

    // BGR to HSV and find binary mask with HSV limits
    cv::cvtColor(bgr_ptr->image, hsv_img, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_img,
                cv::Scalar(LOW_H, LOW_S, LOW_V),
                cv::Scalar(HIGH_H, HIGH_S, HIGH_V),
                b_mask);


    // Define Erosion operation
    er_size = 6;
    element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                         cv::Size( 2*er_size + 1, 2*er_size+1 ),
                                         cv::Point( er_size, er_size ) );
    // Apply the erosion operation
    cv::erode( b_mask, b_mask, element );
    cv::dilate( b_mask, b_mask, element );

    //----------------Improve b_mask with depth data------------------//
    cv::findContours( b_mask, contours, hierarchy,
                      cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );
    std::vector<std::vector<cv::Point> > contours_poly_init( contours.size() );
    std::vector<cv::Rect> boundRectInit( contours.size() );

    for (int i=0; i < contours.size(); i++ ){
      cv::approxPolyDP( cv::Mat(contours[i]), contours_poly_init[i], 3, true );
      boundRectInit[i] = cv::boundingRect( cv::Mat(contours_poly_init[i]) );

      //Define bounding limits
      int x_init=boundRectInit[i].tl().x;
      int y_init=boundRectInit[i].tl().y;
      int x_fin=boundRectInit[i].br().x;
      int y_fin=boundRectInit[i].br().y;
      bool f_flag = false;     //For exiting flag
      //Search for a depth value corresponding with a point of the b_mask
      for( icont = x_init; icont < x_fin; icont++ ){
        for( jcont = y_init; jcont < y_fin; jcont++ ){
          if(b_mask.at<int>(jcont,icont)) {
            actual_float = depth_ptr->image.at<float>(jcont, icont);
            if(actual_float>0 && actual_float<100) {f_flag=true; break;}
          }
        }
        if(f_flag) break;
      }

      //Get the binary mask of the objects at a given distance from depth image
      cv::inRange(depth_ptr->image,
                  actual_float-DEPTH_TOLERANCE,
                  actual_float+DEPTH_TOLERANCE,
                  b_mask_aux);

      //Create mask to eliminate objects out of the corresponding bounding box
      cv::Mat mask = cv::Mat::zeros( msg->height, msg->width, CV_8UC1); // all 0

      //If the bounding rect is in the limit do not expand the bounds
      if( x_init-EXP_COEF < 0 || y_init-EXP_COEF < 0 ||
          x_fin+EXP_COEF > 2*V_CENTER || y_fin+EXP_COEF >2*H_CENTER){
        cv::rectangle(mask, cv::Point(x_init, y_init), cv::Point(x_fin, y_fin), cv::Scalar(1), -1); //-1 = FILLED
      }
      //Expand the bounding limits to get the full object
      else{
        cv::rectangle(mask, cv::Point(x_init-EXP_COEF,y_init-EXP_COEF),
                     cv::Point(x_fin+EXP_COEF,y_fin+EXP_COEF), cv::Scalar(1), -1); //-1 = FILLED
      }

      b_mask_aux = b_mask_aux.mul(mask);
      // Detect Edges
      // Canny( bgr[2].mul(mask), edges, EDGE_THRESHOLD,EDGE_THRESHOLD*EDGE_RATIO, EDGE_OPENING_SIZE );

      morphologyEx( b_mask_aux, b_mask_aux, cv::MORPH_OPEN, element );
      b_mask_combined = b_mask_combined + b_mask_aux + b_mask ;
    }
    

    // Multiply the bin.at(ary mask and the depth image to get approximate distance
    b_mask_combined.convertTo(b_mask_float, CV_32FC1, 1/255.0);
    dist=depth_ptr->image.mul(b_mask_float);

    //----------------Get contours and bounding box------------------//

    // Find contours
    cv::findContours( b_mask_combined, contours, hierarchy,
                      cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );

    /// Approximate contours to polygons + get bounding
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Rect> aerealBoundRect( contours.size() );
    std::vector<float> objDist( contours.size(), 0.0 );
    std::vector<geometry_msgs::Point> objPoseEst(contours.size());

    int n_objects = 0;

    for( int i = 0; i < contours.size(); i++ ){

      //Defined precision polygon approximation
      cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
      boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
      cv::Scalar color( i*25, (contours.size()-i)*25, i*25 );

    //--------------Get estimate distance to objects-----------------//

      bool flag=true;

      //Discard small objects
      if (boundRect[i].height < MIN_OBJ_HEIGH && boundRect[i].width < MIN_OBJ_WIDTH){
        flag = false;
      }

      //Get a small rectangle centered inside the boundRect to avoid distance errors
      int dist_values;
      int x_init=boundRect[i].tl().x+boundRect[i].width/4;
      int y_init=boundRect[i].tl().y+boundRect[i].height/4;
      int x_fin=boundRect[i].br().x-boundRect[i].width/4;
      int y_fin=boundRect[i].br().y-boundRect[i].height/4;

      if (flag){
        dist_values=0;

        //Iterate through contour
        for( icont = x_init; icont < x_fin; icont++ ){
          for( jcont = y_init; jcont < y_fin; jcont++ ){

            //Get the distance value
            actual_float = dist.at<float>(jcont, icont);

            //Chech for valid distance value
            if ( actual_float > 0.0 && actual_float < 100.0){
              objDist[i] = objDist[i] + actual_float;
              dist_values ++;
            }

          }
        }
        // Get the average distance of the object i
        objDist[i] = objDist[i]/dist_values;
        // printf("Distancia del objeto %d: %f\r\n", i, objDist[i]);
        // printf("Points token %d: %d\r\n", i, dist_values);

        //Calculate camera_relative angle of the object i
        float rot_z=(x_init+(boundRect[i].width/2.0)-H_CENTER)*HAP;
        float rot_y=(y_init+(boundRect[i].height/2.0)-V_CENTER)*VAP;
        tf::Vector3 ObjectTranslation(objDist[i],0,0);
        tf::Quaternion ObjectRotation(0,rot_y,rot_z);
        tf::Transform C_I_Transform(ObjectRotation,ObjectTranslation);
        C_I_Transform.mult(W_C_Transform,C_I_Transform);
        ObjectTranslation=C_I_Transform.getOrigin();
        objPoseEst[n_objects].x=ObjectTranslation.getX();
        objPoseEst[n_objects].y=ObjectTranslation.getY();
        objPoseEst[n_objects].z=ObjectTranslation.getZ();
        n_objects ++;

        printf("X[%d]: %f   Y[%d]: %f   Z[%d]: %f   \r\n", i, objPoseEst[i].x, i, objPoseEst[i].y, i, objPoseEst[i].z );
        //Drawing contours
        cv::drawContours( drw, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        cv::rectangle( drw, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
      }
   }
   publishMarkers(n_objects, objPoseEst);
   //------------Transform image to msg and publish it------------//
    img_bridge = cv_bridge::CvImage(h, "8UC3" ,drw);
    img_bridge.toImageMsg(img_msg);
      // Output modified video stream
    image_pub_.publish(img_msg);
  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  image_transport::Publisher image_pub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
  message_filters::Cache<geometry_msgs::PoseStamped> cache;
  // Define sync policy
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image
  > MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync;
  ros::Publisher markers_pub_;
  int lastest_marker_id;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect");
  detection ic;
  while(ros::ok()){
    ros::spin();
  }
  return 0;
}
