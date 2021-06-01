#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <set>
//#include <Eigen/Geometry>
//#include <Eigen/Core>
#include <Vector3.h>

std::vector<float> current(3,0);
std::vector<float> old(3,0);
int id=1000;
int tag_id=-1;
int new_tag_id=-2;
std::set<int> tags;
geometry_msgs::TransformStamped transformStamped;

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
    // RICORDATI DI FARE UN CICLO SUGLI ID
    new_tag_id = msg->detections[0].id[0];
    if(!tags.count(new_tag_id)){
      int i=1000000000;
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      //Eigen::Transform<float,3,Eigen::Affine,Eigen::AutoAlign> t;
      while (i>=0){
        try{
          // DA GUARDARE
          transformStamped = tfBuffer.lookupTransform("fisheye_rect","odom", ros::Time(0));
          //t.translation() << msg->detections[0].pose.pose.pose.position.x, msg->detections[0].pose.pose.pose.position.y, msg->detections[0].pose.pose.pose.position.z;
          //t.translation() << transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z;
          tf2::Quaternion q = tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
          //tf2::Matrix3x3 m(q);
          //double roll, pitch, yaw;
          //m.getRPY(roll, pitch, yaw);
          // devo mettere la rotazione della trasformata nella trasformata e dopo moltiplicare la trasformata per il punto fornito da apriltag
          tf2::Vector3 translation = tf2::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
          tf2::Transform::Transform transf = tf2::Transform::Transform(q, translation);


          
          // tf2::Quaternion q(msg->detections[0].pose.pose.pose.orientation.x, msg->detections[0].pose.pose.pose.orientation.y, msg->detections[0].pose.pose.pose.orientation.z, msg->detections[0].pose.pose.pose.orientation.w);
          // tf2::Matrix3x3 m(q);
          // double roll, pitch, yaw;
          // m.getRPY(roll, pitch, yaw);
          // t.rotate(Eigen::AngleAxis<float>(yaw, Eigen::Vector3f::UnitX()));
          // 
          // float c = cos(); float s = sin();
          // t.linear() = c, -s, s,  c;
          //Eigen::Vector3f point; point << transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z;
          //Eigen::Vector3f point; point << msg->detections[0].pose.pose.pose.position.x, msg->detections[0].pose.pose.pose.position.y, msg->detections[0].pose.pose.pose.position.z;
          //Eigen::Vector3f pointw = t*point;
          tf2::Vector3 point = tf2::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
          tf2::Vector3 pointw = transf*point;
          float xw = pointw.x();
          float yw = pointw.y();
          // perché yw è sempre 0?
          ROS_INFO("VERTEX_XY %d %f %f", new_tag_id, xw ,yw);
          break;
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        i--; 
      }
      tags.insert(new_tag_id);
    }
    tag_id = new_tag_id;
    ROS_INFO("EDGE_SE2_XY %d %d %f %f", id, tag_id, msg->detections[0].pose.pose.pose.position.x, msg->detections[0].pose.pose.pose.position.y); 
  }
}
// voglio la trasformata da fisheye_rect a odom
int main(int argc, char** argv){
  ros::init(argc, argv, "slam_node");
  ros::NodeHandle n;
  

  ros::Subscriber sub_odom = n.subscribe("odom", 1000, odometryCallback);
  ros::Subscriber sub_tag = n.subscribe("tag_detections", 1000, tagCallback);

   ros::Rate rate(10.0);
  while (n.ok()){
    ros::spinOnce();
    

    
   rate.sleep();
  }
  return 0;
};
