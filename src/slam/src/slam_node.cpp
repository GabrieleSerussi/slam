#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <apriltag_ros/AprilTagDetection.h>

std::vector<float> current(3,0);
std::vector<float> old(3,0);
int id=1000;
int tag_id=-1;

void odometryCallback(const nav_msgs::Odometry::ConstPtr msg) {
  
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current[0]=msg->pose.pose.position.x;
  current[1]=msg->pose.pose.position.y;
  current[2]=yaw;

  if(old[0]==0 && old[1]==0 && old[2]==0){
    old=current;
    ROS_INFO("VERTEX_SE2 %d %f %f %f",id, current[0],current[1],current[2]);
  }
  else{
    float distance_position= sqrt(pow(current[0]-old[0],2)+pow(current[1]-old[1],2));
    float distance_orientation= current[2]-old[2];
    if(distance_position >= 0.1 || distance_orientation >= 0.5){
      id++;
      ROS_INFO("VERTEX_SE2 %d %f %f %f",id, current[0],current[1],current[2]);
      ROS_INFO("EDGE_SE2 %d %d %f %f %f",id-1, id, current[0]-old[0],current[1]-old[1],current[2]-old[2]);
      old=current;
    }
  }
}
void tagCallback(const apriltag_ros::AprilTagDetection::ConstPtr msg) {
  tag_id = msg->id[0];
  ROS_INFO("EDGE_SE2_XY %d %d %f %f", id, tag_id, msg->pose.pose.pose.position.x, msg->pose.pose.pose.position.y);
}
// voglio la trasformata da fisheye_rect a odom
int main(int argc, char** argv){
  ros::init(argc, argv, "slam_node");

  ros::NodeHandle n;

  ros::Subscriber sub_odom = n.subscribe("odom", 1000, odometryCallback);
  ros::Subscriber sub_tag = n.subscribe("tag_detections", 1000, tagCallback);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (n.ok()){
    ros::spinOnce();
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("odom", "fisheye_rect",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    ROS_INFO("VERTEX_XY %d %f %f",tag_id, transformStamped.transform.translation.x,transformStamped.transform.translation.y);
   rate.sleep();
  }
  return 0;
};