#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include <math.h>
#include <gazebo_msgs/SpawnModel.h>
#include <sstream>
#include <fstream>
#include <string>
#include <urdf/model.h>


/*-------------------------------------------*/
std::string intToString (int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
 }
/*-------------------------------------------*/
bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}
/*-------------------------------------------*/



int main(int argc, char **argv)
{
  ros::init(argc, argv, "spawn_arena");
  ros::NodeHandle nh;
  //service client for service /gazebo/spawn_urdf_model
  ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
  gazebo_msgs::SpawnModel::Request spawn_model_req;
  gazebo_msgs::SpawnModel::Response spawn_model_resp;
  // ros::Rate loop_rate(10);
  int N_BALLOONS;
  std::string MODEL_PATH;

  if(nh.param<int>("/detection/spawn_params/n_balloons", N_BALLOONS, 15) &&
     nh.param<std::string>("/detection/spawn_params/model_path", MODEL_PATH,
                           "/home/fidel/catkin_ws/src/DeTCT/urdf/fixed_balloon_shiny.sdf")){
    ROS_INFO("Parameters received");
  }
  else {
    ROS_ERROR("Error receiving the parameters");
  }

  // ROS_INFO()

  bool service_ready = false;
  while (!service_ready){
    service_ready = ros::service::exists("gazebo/spawn_sdf_model", true);
    ROS_INFO("waiting for spawn_model service");
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("spawn_urdf_model service is ready");

  int count = 0;

  std::ifstream balloon_inXml;
  std::stringstream balloon_strStream;
  std::string balloon_xmlStr;
  std::string PATH = "/home/fidel/catkin_ws/src/DeTCT/urdf/fixed_balloon_shiny.sdf";
  balloon_inXml.open(PATH.c_str());
  balloon_strStream << balloon_inXml.rdbuf();
  balloon_xmlStr = balloon_strStream.str();

  // ROS_INFO_STREAM("urdf: \n" <<balloon_xmlStr);

  while (count < N_BALLOONS)
  {
    std::string index = intToString(count);
    std::string model_name;
    std::string randStr;

    randStr = "0."+intToString((float)rand()/(float)(RAND_MAX)*100.0);
    ROS_INFO_STREAM(randStr);
    replace(balloon_xmlStr,"%R",randStr);
    randStr = "0."+intToString((float)rand()/(float)(RAND_MAX)*100.0);
    ROS_INFO_STREAM(randStr);
    replace(balloon_xmlStr,"%G",randStr);
    randStr = "0."+intToString((float)rand()/(float)(RAND_MAX)*100.0);
    ROS_INFO_STREAM(randStr);
    replace(balloon_xmlStr,"%B",randStr);

    spawn_model_req.initial_pose.position.x = (float)rand()/(float)(RAND_MAX) * 100-50;
    spawn_model_req.initial_pose.position.y = (float)rand()/(float)(RAND_MAX) * 60-30;
    ROS_INFO_STREAM("[X, Y] Balloon " << count << " : "
    << spawn_model_req.initial_pose.position.x << "   " << spawn_model_req.initial_pose.position.y);

    model_name = "Balloon_" + index;  // initialize model_name
    spawn_model_req.model_name = model_name;
    spawn_model_req.robot_namespace = model_name;
    spawn_model_req.model_xml = balloon_xmlStr;
    // spawn_model_req.reference_frame = "map";

    bool call_service = spawnClient.call(spawn_model_req, spawn_model_resp);

    if (call_service) {
      if (spawn_model_resp.success) {
        ROS_INFO_STREAM(model_name << " has been spawned");
      }
      else {
        ROS_INFO_STREAM(model_name << " spawn failed");
      }
    }
    else {
      ROS_INFO("fail in first call");
      ROS_ERROR("fail to connect with gazebo server");
      return 0;
    }

    // ros::spinOnce();
    // loop_rate.sleep();
    ++count;
  }


  return 0;
}
