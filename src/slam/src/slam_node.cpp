#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <set>
#include <tf2/LinearMath/Transform.h>
#include <iostream>
#include <fstream>

std::ofstream file; 

std::vector<float> current(3,0);
std::vector<float> old;
int id=1000;
int tag_id=-1;
int new_tag_id=-2;
std::set<int> tags;
geometry_msgs::TransformStamped transformStamped;

void odometryCallback(const nav_msgs::Odometry::ConstPtr msg) {
  
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // restituisce un angolo in radianti compreso tra -π e π
  double roll, pitch, yaw; tf2::Matrix3x3 m(q); m.getRPY(roll, pitch, yaw);
  // il messaggio fornisce la posizione in metri
  current[0]=msg->pose.pose.position.x; current[1]=msg->pose.pose.position.y; current[2]=yaw;

  if(old.empty()){
    old=current;
    file << "VERTEX_SE2 " << id << " " << current[0] << " " << current[1] << " " << current[2] << "\n";
  }
  else{
    float traslation= sqrt(pow(current[0]-old[0],2)+pow(current[1]-old[1],2));
    float rotation= current[2]-old[2];
    if(traslation >= 0.1 || rotation >= 0.5){
      id++;
      file << "VERTEX_SE2 " << id << " " << current[0] << " " << current[1] << " " << current[2] << "\n";
      file << "EDGE_SE2 " << id-1 << " " << id << " " << current[0]-old[0] << " " << current[1]-old[1] << " " << current[2]-old[2] << "\n";
      old=current;
    }
  }
}
/*
apriltag_ros/ApriTagDetectionArray

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
apriltag_ros/AprilTagDetection[] detections
  int32[] id
  float64[] size
  geometry_msgs/PoseWithCovarianceStamped pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/PoseWithCovariance pose
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      float64[36] covariance
*/
void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr msg) {
  if (!msg->detections.empty()){  
    for(int i=0; i<msg->detections.size(); i++){
      for(int j=0; j<msg->detections[i].id.size(); j++){
        new_tag_id = msg->detections[i].id[j];
        // posizione del tag rispetto alla fotocamera del robot
        tf2::Vector3 point = tf2::Vector3(msg->detections[i].pose.pose.pose.position.x, msg->detections[i].pose.pose.pose.position.y, msg->detections[i].pose.pose.pose.position.z);
      
        if(!tags.count(new_tag_id)){
          tf2_ros::Buffer tfBuffer;
          tf2_ros::TransformListener tfListener(tfBuffer);

          try{
            // prendo la trasformata dalla fotocamera a odom
            transformStamped = tfBuffer.lookupTransform("odom","fisheye_rect", ros::Time(0), ros::Duration(3.0));
            tf2::Quaternion q = tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            tf2::Vector3 translation = tf2::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            tf2::Transform transf_camera2odom = tf2::Transform(q, translation);

            // posizione del tag rispetto all'odometria
            tf2::Vector3 pointw = transf_camera2odom*point;
            float xw = pointw.x();
            float yw = pointw.y();
            file << "VERTEX_XY " << new_tag_id << " " << xw << " " << yw << "\n";
          }
          catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
          }
          tags.insert(new_tag_id);
        }
        tag_id = new_tag_id;
        // trasformata dalla posizione corrente alla posizione vecchia
        tf2::Vector3 vec = tf2::Vector3(current[0]-old[0], current[1]-old[1], 0);
        tf2::Quaternion quat; quat.setRPY(0,0,current[2]-old[2]);
        tf2::Transform transf_current2old = tf2::Transform(quat, vec);
        
        tf2::Vector3 pointr = transf_current2old * point;
        float xr = pointr.x();
        float yr = pointr.y();

        file << "EDGE_SE2_XY " << id << " " << tag_id << " " << xr << " " << yr << "\n";
      }
    }
  }
}
int main(int argc, char** argv){
  ros::init(argc, argv, "slam_node");
  ros::NodeHandle n;
  ros::Subscriber sub_odom = n.subscribe("odom", 1000, odometryCallback);
  ros::Subscriber sub_tag = n.subscribe("tag_detections", 1000, tagCallback);

  file.open("out.g2o");
  ros::Rate rate(10.0);
  while (n.ok()){
    ros::spinOnce();
    rate.sleep();
  }
  file.close();
  return 0;
};
