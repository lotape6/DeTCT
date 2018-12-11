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

#define MIN_OBJ_HEIGH 3
#define MIN_OBJ_WIDTH 3

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
#define HAP 0.00158170468
#define VAP 0.00163624625
#define H_CENTER 320
#define V_CENTER 240

namespace enc = sensor_msgs::image_encodings;

class detection
{
public:
  detection()
    : it_(nh_),
    // Subscribe to RGB img, depth img and to satamped pose
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
  }

  ~detection()
  {
  }

  // Define the image [rgb, depth] callback (which also takes position)
  void imageCb(const sensor_msgs::ImageConstPtr& msg,
               const sensor_msgs::ImageConstPtr& depth_msg)  {
    //-----------------------Define variables-----------------------//
    //Color Limits (HSV)
    uint8_t low_H, low_S, low_V, high_H, high_S, high_V;

<<<<<<< HEAD
    //CV_Bridge
    cv_bridge::CvImage img_bridge;

    //ROS image msg
    sensor_msgs::Image img_msg; // >> message to be sent
    cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
=======
    low_H = 7;     //¿ORANGE?
    high_H = 30;

    low_S = 100;     //Else White or gray
    high_S = 255;
>>>>>>> 16f0176f0ed0a765b34fe5f995eca8582e41e9f2

    //Euler angles and position of the camera
    double roll, pitch, yaw;
    float x, y, z;

    //OpenCV images
    cv::Mat b_mask = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);
    cv::Mat b_mask_float = cv::Mat::zeros(msg->height, msg->width, CV_32FC1);
    cv::Mat drw = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);
    cv::Mat dist= cv::Mat::zeros(msg->height, msg->width, CV_32FC1);
    cv::Mat hsv_img;

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
      rgb_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      depth_ptr = cv_bridge::toCvCopy(depth_msg, "32FC1");
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //------------Get binary mask and perform a opening---------------//
    // Define HSV limits for color segmentation
    low_H = 7;    high_H = 45;     //ORANGE
    low_S = 110;  high_S = 255;    //Else White or gray
    low_V = 64;   high_V = 255;    //Else Black


    // BGR to HSV and find binary mask with HSV limits
    cv::cvtColor(rgb_ptr->image, hsv_img, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_img,
                cv::Scalar(low_H, low_S, low_V),
                cv::Scalar(high_H, high_S, high_V),
                b_mask);

    // Define Erosion operation
<<<<<<< HEAD
    er_size = 6;
    element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                         cv::Size( 2*er_size + 1, 2*er_size+1 ),
                                         cv::Point( er_size, er_size ) );
    // Apply the erosion operation
    cv::erode( b_mask, b_mask, element );
    cv::dilate( b_mask, b_mask, element );

    // Multiply the bin.at(ary mask and the depth image to get approximate distance
    b_mask.convertTo(b_mask_float, CV_32FC1, 1/255.0);
    dist=depth_ptr->image.mul(b_mask_float);

    //----------------Get contours and bounding box------------------//

    // Find contours
    cv::findContours( b_mask, contours, hierarchy,
                      cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );

    /// Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Rect> aerealBoundRect( contours.size() );
    std::vector<cv::Point2f>center( contours.size() );
    std::vector<float> objDist( contours.size(), 0.0 );
    std::vector<geometry_msgs::Point> objPoseEst(contours.size());

    for( int i = 0; i < contours.size(); i++ ){

      //Defined precision polygon approximation
      cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
      boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
      cv::Scalar color( i*25, (contours.size()-i)*25, i*25 );

      //Calculate estimated height

      bool flag=true;
=======
    int erosion_size = 4;
    int dilation_size = 8;
    int opening_size = 5;

    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*opening_size + 1, 2*opening_size+1 ),
                                           cv::Point( opening_size, opening_size ) );
    // Execute opening operation
    cv::morphologyEx(b_mask, b_mask, cv::MORPH_OPEN, element);
>>>>>>> 16f0176f0ed0a765b34fe5f995eca8582e41e9f2

      //Discard small objects
      if (boundRect[i].height < MIN_OBJ_HEIGH && boundRect[i].width < MIN_OBJ_WIDTH){
        flag = false;
      }

      int icont, jcont;
      float actual_float;
      int dist_values;
      int x_init=boundRect[i].tl().x;
      int y_init=boundRect[i].tl().y;
      int x_fin=boundRect[i].br().x;
      int y_fin=boundRect[i].br().y;

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
      }
      //Calculate camera_relative angle of the object i
      float rot_z=(x_init+(boundRect[i].width/2.0)-H_CENTER)*HAP;
      float rot_y=(y_init+(boundRect[i].height/2.0)-V_CENTER)*VAP;
      tf::Vector3 ObjectTranslation(objDist[i],0,0);
      tf::Quaternion ObjectRotation(0,rot_y,rot_z);
      tf::Transform C_I_Transform(ObjectRotation,ObjectTranslation);
      C_I_Transform.mult(W_C_Transform,C_I_Transform);
      ObjectTranslation=C_I_Transform.getOrigin();
      objPoseEst[i].x=ObjectTranslation.getX();
      objPoseEst[i].y=ObjectTranslation.getY();
      objPoseEst[i].z=ObjectTranslation.getZ();

      printf("X[%d]: %f   Y[%d]: %f   Z[%d]: %f   \r\n", i, objPoseEst[i].x, i, objPoseEst[i].y, i, objPoseEst[i].z );
      //Drawing contours
      cv::drawContours( drw, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
      cv::rectangle( drw, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
   }

   //------------Transform image to msg and publish it------------//
    img_bridge = cv_bridge::CvImage(h, "32FC1" ,dist);
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


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect");
  detection ic;
  ros::spin();
  return 0;
}
