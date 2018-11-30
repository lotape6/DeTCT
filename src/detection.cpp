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
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>


namespace enc = sensor_msgs::image_encodings;

class detection
{
public:
  detection()
    : it_(nh_),
    // Subscrive to input video feed and publish output video feed
    image_sub_(nh_,"/camera/rgb/image_raw", 1),
    depth_sub_(nh_,"/camera/depth/image", 80),
    pose_sub_(nh_,"/vicon_client/AsusXtionPro/pose",80),
    sync(MySyncPolicy(10),image_sub_, depth_sub_,pose_sub_)
{

    image_pub_ = it_.advertise("/image_converter/output_video", 5);
    sync.registerCallback(boost::bind(&detection::imageCb,this, _1, _2, _3));
  }

  ~detection()
  {
  }




  void imageCb(const sensor_msgs::ImageConstPtr& msg,
               const sensor_msgs::ImageConstPtr& depth_msg,
               const geometry_msgs::PoseStampedConstPtr& p)

  {
    printf("HOLAPUTO");
    double roll, pitch, yaw;
    tf::Quaternion quat(p->pose.orientation.x,
                        p->pose.orientation.y,
                        p->pose.orientation.z,
                        p->pose.orientation.w);

    //Transform quaternions to Euler angles
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM("X: "<<p->pose.position.x<<" , Y: "<<p->pose.position.y<<" , Z: "<<p->pose.position.z);
    ROS_INFO_STREAM("Roll: "<<roll<<" , Pitch: "<<pitch<<" , Yaw: "<<yaw);


    //Color Limits (HSV)
    uint8_t low_H, low_S, low_V, high_H, high_S, high_V;

    low_H = 7;     //¿ORANGE?
    high_H = 45;

    low_S = 110;     //Else White or gray
    high_S = 255;

    low_V = 64;     //Else Black
    high_V = 255;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat hsv_img;

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat b_mask(cv_ptr->image.size(),cv_ptr->image.type());
    cv::Mat drw = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
    cv::Mat dist= cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);

    // BGR to HSV
    cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);


    cv::inRange(hsv_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), b_mask);

    // Define Erosion operation
    int erosion_size = 5;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                           cv::Point( erosion_size, erosion_size ) );
    /// Apply the erosion operation
    cv::erode( b_mask, b_mask, element );
    cv::dilate( b_mask, b_mask, element );

    //--------------------Contours---------------------//

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( b_mask, contours, hierarchy,
        cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    /// Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Point2f>center( contours.size() );
    // std::vector<float>radius( contours.size() );
    /// Draw polygonal contour + bonding rects + circles

    for( int i = 0; i < contours.size(); i++ )
   {
       //Defined precision polygon approximation
       cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
       cv::Scalar color( i*25, (contours.size()-i)*25, i*25 );
       // cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );

       //Drawing contours
       cv::drawContours( drw, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
       cv::rectangle( drw, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
       // cv::circle( drw, center[i], (int)radius[i], color, 2, 8, 0 );
   }

   // if(depth_written){
     // dist=depth_image_ptr->image.mul(b_mask);

   // }

    std_msgs::Header header; // empty header
    header = cv_ptr->header; // user defined counter

    img_bridge = cv_bridge::CvImage(header, enc::MONO8 ,drw);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      // Output modified video stream
    image_pub_.publish(img_msg); //cv_ptr
  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  image_transport::Publisher image_pub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;

  // Define sync policy
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped
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
