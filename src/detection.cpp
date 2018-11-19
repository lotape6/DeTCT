#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

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

    // PERFORM DETECTION

    cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    std_msgs::Header header; // empty header
    header = cv_ptr->header; // user defined counter

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, hsv_img);
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
