#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>

class detection
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber sub;

public:
  detection()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &detection::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    //                    ,&detection::poseH,this);
    sub = nh_.subscribe("/vicon_client/AsusXtionPro/pose",1
                        ,&detection::poseH,this);
  }

  ~detection()
  {
  }

  void poseH(const geometry_msgs::PoseStamped& p){
    ROS_INFO_STREAM("X: "<<p.pose.position.x<<" , Y: "<<p.pose.position.y<<" , Z: "<<p.pose.position.z);
    geometry_msgs::Quaternion q=p.pose.orientation;
    double roll, pitch, yaw;
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
  	double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  	roll = atan2(sinr_cosp, cosr_cosp);

  	// pitch (y-axis rotation)
  	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  	if (fabs(sinp) >= 1)
  		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  	else
  		pitch = asin(sinp);

  	// yaw (z-axis rotation)
  	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
  	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  	yaw = atan2(siny_cosp, cosy_cosp);

    ROS_INFO_STREAM("Roll: "<<roll<<" , Pitch: "<<pitch<<" , Yaw: "<<yaw);


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
    cv::Mat dst = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);

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



    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( b_mask, contours, hierarchy,
        cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    //Draw contours
    //int idx = 0;
    //for( ; idx >= 0; idx = hierarchy[idx][0] )
    int idx;
    for(idx=0;idx<contours.size();idx++)
    {
        cv::Scalar color( idx*25, (contours.size()-idx)*25, idx*25 );
        cv::drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
    }


    std_msgs::Header header; // empty header
    header = cv_ptr->header; // user defined counter

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8  , dst);
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
