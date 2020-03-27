#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <string>
#include <math.h>

#define rad_to_deg(rad) (((rad)/2/M_PI)*360)

int main(int argc, char **argv){
  ros::init(argc, argv, "joint_torque_publisher");
  ros::NodeHandle nh;

  // get joint names
  std::vector<std::string> joint_names;
  nh.getParam("/crane_x7/arm_controller/joints", joint_names);

  // action for command 
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/crane_x7/arm_controller/follow_joint_trajectory", true);
  if (!ac.waitForServer(ros::Duration(3.0)))
  {
    ROS_ERROR_STREAM("Cannot found action server");
    return 1;
  }

  trajectory_msgs::JointTrajectory jt;
  jt.joint_names = joint_names;
  jt.points.position = rad_to_deg(-45)
  //jt.points.effort 
  //jt.points.

  ros::Rate loop_rate(10);
  int count=0;
  while (ros::ok()){
    sensor_msgs::JointState js0;
    js0.header.stamp = ros::Time::now();
    js0.name.resize(2);
    js0.name[0]="crane_x7_shoulder_fixed_part_pan_joint";
    js0.name[1]="crane_x7_shoulder_revolute_part_tilt_joint";
    js0.position.resize(2);
    js0.position[0]=-1.0*(float)count/40.0;
    js0.position[1]= 2.0*(float)count/40.0;
    joint_pub.publish(js0);
    count++;

    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
