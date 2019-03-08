#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <iostream>
#include <boost/bind.hpp>

void callback (const visualization_msgs::MarkerArrayConstPtr& MarkerArray);

ros::Publisher balloons_markers_pub;
float x1Balloon, y1Balloon, z1Balloon, x2Balloon, y2Balloon, z2Balloon;

int main(int argc, char** argv){
  ros::init(argc, argv, "balloons_viz");
  ros::NodeHandle nh_;
  ros::Subscriber sub = nh_.subscribe("/detct/estimated_objects_markers", 10, callback);
  balloons_markers_pub = nh_.advertise<visualization_msgs::Marker>("/balloon_viz/exact_balloons_pos", 1);
  x1Balloon = -0.300653;
  y1Balloon = 2.97126;
  z1Balloon = 1.60321;

  x2Balloon = -1.69197;
  y2Balloon = 0.55412;
  z2Balloon = 1.11892;



  while(ros::ok()){
      ros::spin();
  }
  return 0;
}


void callback (const visualization_msgs::MarkerArrayConstPtr& MarkerArray){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "vicon";
  marker.header.stamp = ros::Time();
  marker.ns = "balloon_viz";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x1Balloon;
  marker.pose.position.y = y1Balloon;
  marker.pose.position.z = z1Balloon;
  marker.scale.x = 0.554;
  marker.scale.y = 0.554;
  marker.scale.z = 0.9;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.b = 1.0;
  balloons_markers_pub.publish(marker);

  marker.id = 1;
  marker.pose.position.x = x2Balloon;
  marker.pose.position.y = y2Balloon;
  marker.pose.position.z = z2Balloon;

  balloons_markers_pub.publish(marker);


}
