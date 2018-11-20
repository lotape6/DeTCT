#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>



class detection
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  detection()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &detection::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

  }

  ~detection()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    namespace enc = sensor_msgs::image_encodings;

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

    // BGR to HSV
    cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);


    cv::inRange(hsv_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), b_mask);


    // Draw an example circle on the video stream

    std_msgs::Header header; // empty header
    header = cv_ptr->header; // user defined counter

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8  , b_mask);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      // Output modified video stream
    image_pub_.publish(img_msg); //cv_ptr
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect");
  detection ic;
  ros::spin();
  return 0;
}
