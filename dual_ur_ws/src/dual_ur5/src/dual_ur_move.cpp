#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include "ur_kinematics/ur_kin.h"
#include "ur_kinematics/ikfast.h"
#include <Eigen/Dense>

using namespace ikfast;
using namespace ur_kinematics;
sensor_msgs::JointState RightJoint;

// Our Action interface type for moving UR5, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

// Create an arm controller action client to move the UR5's arm
 arm_control_client_Ptr LeftArmClient,RightArmClient;
 control_msgs::FollowJointTrajectoryGoal left_arm_goal,right_arm_goal;


 void createArmClient(arm_control_client_Ptr& actionClient,std::string ArmName)
 {
  ROS_INFO("Creating action client to arm controller ...");
   if(ArmName=="right")
   {
     actionClient.reset( new arm_control_client("right_arm_controller/follow_joint_trajectory") );
   }
   else if(ArmName=="left"){
      actionClient.reset( new arm_control_client("left_arm_controller/follow_joint_trajectory") );
   }

     int iterations = 0, max_iterations = 3;
     // Wait for arm controller action server to come up
     while( !actionClient->waitForServer(ros::Duration(2)) && ros::ok() && iterations < max_iterations )
   {
       ROS_DEBUG("Waiting for the arm_controller_action server to come up");
       ++iterations;
     }

     if ( iterations == max_iterations )
       throw std::runtime_error("Error in createArmClient: arm controller action server not available");
  }


   // Generates a simple trajectory with two waypoints to move TIAGo's arm
   void waypoints_left_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
  {
     // The joint names, which apply to all waypoints
     goal.trajectory.joint_names.push_back("left_shoulder_pan_joint");
     goal.trajectory.joint_names.push_back("left_shoulder_lift_joint");
     goal.trajectory.joint_names.push_back("left_elbow_joint");
     goal.trajectory.joint_names.push_back("left_wrist_1_joint");
     goal.trajectory.joint_names.push_back("left_wrist_2_joint");
     goal.trajectory.joint_names.push_back("left_wrist_3_joint");

     size_t PointNum=360;
     // Two waypoints in this goal trajectory
     goal.trajectory.points.resize(PointNum);

     // First trajectory point
    // Positions
     for (size_t theta=0;theta<PointNum;theta++) {
        double r=0.1,x=0,y=0.4,z=0.4;
        x=x+r*cos(theta*3.1415926/180);
        y=y+r*sin(theta*3.1415926/180);
        Eigen::Matrix4d TempMatrix;
        TempMatrix <<0, -0.707, -0.707,x,
            0,-0.707,0.707,y,
            -1,0,0,z,
            0,0,0,1;

        double* T = new double[16];
        for(int i=0;i<4;i++) {
               for(int j=i*4;j<(i+1)*4;j++)
                 T[j]=TempMatrix(i,j%4);
             }
        double q_sols[8*6];
        int num_sols;
        num_sols = inverse(T, q_sols);
        goal.trajectory.points[theta].positions.resize(6);
        goal.trajectory.points[theta].positions[0] = q_sols[12+0];
        goal.trajectory.points[theta].positions[1] = q_sols[12+1];
        goal.trajectory.points[theta].positions[2] = q_sols[12+2];
        goal.trajectory.points[theta].positions[3] = q_sols[12+3];
        goal.trajectory.points[theta].positions[4] = q_sols[12+4];
        goal.trajectory.points[theta].positions[5] = q_sols[12+5];
        // Velocitiess
        goal.trajectory.points[theta].velocities.resize(6);
        for (size_t j = 0; j < 6; ++j){
         goal.trajectory.points[theta].velocities[j] = 0.0;
        }
       // To be reached 2 second after starting along the trajectory
        goal.trajectory.points[theta].time_from_start = ros::Duration((double)theta/20);
     }
 }

   void waypoints_right_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
  {
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("right_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("right_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("right_elbow_joint");
    goal.trajectory.joint_names.push_back("right_wrist_1_joint");
    goal.trajectory.joint_names.push_back("right_wrist_2_joint");
    goal.trajectory.joint_names.push_back("right_wrist_3_joint");
    // One waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    size_t theta = 0;
    goal.trajectory.points[theta].positions.resize(6);
    // Positions
    goal.trajectory.points[theta].positions[0] = RightJoint.position[0];
    goal.trajectory.points[theta].positions[1] = RightJoint.position[1];
    goal.trajectory.points[theta].positions[2] = RightJoint.position[2];
    goal.trajectory.points[theta].positions[3] = RightJoint.position[3];
    goal.trajectory.points[theta].positions[4] = RightJoint.position[4];
    goal.trajectory.points[theta].positions[5] = RightJoint.position[5];
    //Velocities
    goal.trajectory.points[theta].velocities.resize(6);
    for (size_t j = 0; j < 6; ++j){
    goal.trajectory.points[theta].velocities[j] = 0.0;
    }
    // To be reached 0.1 second after starting along the trajectory
    goal.trajectory.points[theta].time_from_start = ros::Duration(0.1);

 }

   void JointCallback(const sensor_msgs::JointState::ConstPtr& msg)
   {
    // Receive the joint angle information of the robot
    RightJoint.position.resize(6);
    RightJoint.position[0]=msg->position[2];
    RightJoint.position[1]=msg->position[1];
    RightJoint.position[2]=msg->position[0];
    RightJoint.position[3]=msg->position[3];
    RightJoint.position[4]=msg->position[4];
    RightJoint.position[5]=msg->position[5];
    //正运动学
    double q[6] = {RightJoint.position[0],RightJoint.position[1],RightJoint.position[2],RightJoint.position[3],RightJoint.position[4],RightJoint.position[5]};
    double* T = new double[16];
    forward(q, T);
    //计算从机器人齐次矩阵
    Eigen::Matrix4d LeftRobotMatrix;
    LeftRobotMatrix <<T[0],T[1],T[2], T[3],
          T[4], T[5], T[6],T[7],
          T[8],T[9],T[10],T[11],
         T[12],T[13],T[14],T[15];

    Eigen::Matrix4d TransMatrix,PosMatrix;
    TransMatrix<<-1,0,0,0.05,
       0,-1,0,0,
       0,0,1,0,
       0,0,0,1;
    PosMatrix<<1,0,0,0,
       0,1,0,-0.8,
       0,0,1,0,
       0,0,0,1;
    LeftRobotMatrix=PosMatrix*LeftRobotMatrix*TransMatrix;
    double q_sols[8*6];
    int num_sols;
    for(int i=0;i<4;i++) {
          for(int j=i*4;j<(i+1)*4;j++)
            T[j]=LeftRobotMatrix(i,j%4);
        }
    num_sols = inverse(T, q_sols);
    RightJoint.position[0]=q_sols[0];
    RightJoint.position[1]=q_sols[1];
    RightJoint.position[2]=q_sols[2];
    RightJoint.position[3]=q_sols[3];
    RightJoint.position[4]=q_sols[4];
    RightJoint.position[5]=q_sols[5];


    // Generates the goal for the right_arm's arm
    waypoints_right_arm_goal(right_arm_goal);
    // Sends the command to start the given trajectory 1s from now
    right_arm_goal.trajectory.header.stamp = ros::Time::now();
    RightArmClient->sendGoal(right_arm_goal);

    //clear right_arm's trajectory
    right_arm_goal.trajectory.points.clear();
    right_arm_goal.trajectory.joint_names.clear();
    right_arm_goal.trajectory.header.frame_id.clear();
   }

   int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_ur_move");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("joint_states", 1000, JointCallback);
    createArmClient(LeftArmClient,"left");
    createArmClient(RightArmClient,"right");
    waypoints_left_arm_goal(left_arm_goal);
    left_arm_goal.trajectory.header.stamp = ros::Time::now();
    LeftArmClient->sendGoal(left_arm_goal);
    ros::spin();
    return 0;

}
