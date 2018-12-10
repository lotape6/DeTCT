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
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>
#include "tf/transform_datatypes.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/cache.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>

#define MIN_OBJ_HEIGH 3
#define MIN_OBJ_WIDTH 3
namespace enc = sensor_msgs::image_encodings;

class detection
{
public:
  detection()
    : it_(nh_),
    // Subscribe to RGB img, depth img and to satamped pose
    image_sub_(nh_,"/camera/rgb/image_raw", 1),
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

    //CV_Bridge
    cv_bridge::CvImage img_bridge;

    //ROS image msg
    sensor_msgs::Image img_msg; // >> message to be sent
    cv_bridge::CvImagePtr rgb_ptr, depth_ptr;

    //Euler angles
    double roll, pitch, yaw;

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
    er_size = 8;
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
    std::vector<cv::Point2f>center( contours.size() );
    std::vector<float> objDist( contours.size(), 0.0 );

    for( int i = 0; i < contours.size(); i++ ){

      //Defined precision polygon approximation
      cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
      boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
      cv::Scalar color( i*25, (contours.size()-i)*25, i*25 );

      //Calculate estimated height

      bool flag=true;

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
        dist_values=1;

        //Iterate through contour
        for( icont = x_init; icont < x_fin; icont++ ){
          for( jcont = y_init; jcont < y_fin; jcont++ ){

            actual_float = dist.at<float>(icont, jcont);  //Get the distance value

              if ( actual_float > 0.0 && actual_float < 1000.0){     //Chech for valid distance value
                objDist[i] = objDist[i] + actual_float;
                dist_values ++;
              }

          }
        }
        objDist[i] = objDist[i]/dist_values;
        printf("Distancia del objeto %d: %f\r\n", i, objDist[i]);
        printf("Points token %d: %d\r\n", i, dist_values);
      }
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
