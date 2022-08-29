#include "ros/ros.h"
#include "std_msgs/String.h"

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>


// Our Action interface type for moving UR5, provided as a typedef for convenience
 typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
 typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

 // Create a ROS action client to move TIAGo's arm
 void createArmClient(arm_control_client_Ptr& actionClient)
 {
  ROS_INFO("Creating action client to arm controller ...");

   actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );

     int iterations = 0, max_iterations = 3;
     // Wait for arm controller action server to come up
     while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
   {
       ROS_DEBUG("Waiting for the arm_controller_action server to come up");
       ++iterations;
     }

     if ( iterations == max_iterations )
       throw std::runtime_error("Error in createArmClient: arm controller action server not available");
   }


   // Generates a simple trajectory with two waypoints to move TIAGo's arm
   void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
  {
     // The joint names, which apply to all waypoints
     goal.trajectory.joint_names.push_back("shoulder_pan_joint");
     goal.trajectory.joint_names.push_back("shoulder_lift_joint");
     goal.trajectory.joint_names.push_back("elbow_joint");
     goal.trajectory.joint_names.push_back("wrist_1_joint");
     goal.trajectory.joint_names.push_back("wrist_2_joint");
     goal.trajectory.joint_names.push_back("wrist_3_joint");


     // Two waypoints in this goal trajectory
     goal.trajectory.points.resize(2);

     // First trajectory point
    // Positions
     size_t index = 0;
     goal.trajectory.points[index].positions.resize(6);
     goal.trajectory.points[index].positions[0] = 0;
     goal.trajectory.points[index].positions[1] = -1.57;
     goal.trajectory.points[index].positions[2] = 1.57;
     goal.trajectory.points[index].positions[3] = -1.57;
     goal.trajectory.points[index].positions[4] = -1.57;
     goal.trajectory.points[index].positions[5] = 0;
     // Velocities
     goal.trajectory.points[index].velocities.resize(6);
     for (size_t j = 0; j < 6; ++j)
     {
       goal.trajectory.points[index].velocities[j] = 1.0;

     }
     // To be reached 2 second after starting along the trajectory
     goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

     // Second trajectory point
     // Positions
     index += 1;
     goal.trajectory.points[index].positions.resize(6);
     goal.trajectory.points[index].positions[0] = 1.57;
     goal.trajectory.points[index].positions[1] = -1.57;
     goal.trajectory.points[index].positions[2] = 1.57;
     goal.trajectory.points[index].positions[3] = -1.57;
     goal.trajectory.points[index].positions[4] = -1.57;
     goal.trajectory.points[index].positions[5] = 0;
   // Velocities
   goal.trajectory.points[index].velocities.resize(6);
   for (size_t j = 0; j < 6; ++j)
   {
     goal.trajectory.points[index].velocities[j] = 0.0;
   }
   // To be reached 4 seconds after starting along the trajectory
   goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
 }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_move");
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
      {
        ROS_FATAL("Timed-out waiting for valid time.");
        return EXIT_FAILURE;
      }

     // Create an arm controller action client to move the UR5's arm
      arm_control_client_Ptr ArmClient;
      createArmClient(ArmClient);

      // Generates the goal for the TIAGo's arm
      control_msgs::FollowJointTrajectoryGoal arm_goal;
      waypoints_arm_goal(arm_goal);

      // Sends the command to start the given trajectory 1s from now
      arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
      ArmClient->sendGoal(arm_goal);

      // Wait for trajectory execution
      while(!(ArmClient->getState().isDone()) && ros::ok())
      {
       ros::Duration(4).sleep(); // sleep for four seconds
      }

      return EXIT_SUCCESS;

}
