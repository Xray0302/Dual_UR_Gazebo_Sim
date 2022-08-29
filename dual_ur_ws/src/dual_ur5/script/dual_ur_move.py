#!/usr/bin/env python
from trajectory_msgs.msg import *
from control_msgs.msg import *
import rospy
import actionlib
import sys
import threading
 
LEFT_JOINT_NAMES = ['left_shoulder_pan_joint', 'left_shoulder_lift_joint', 'left_elbow_joint',
               'left_wrist_1_joint', 'left_wrist_2_joint', 'left_wrist_3_joint']
 
RIGHT_JOINT_NAMES = ['right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint',
               'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']
 
 
def right_plan():
 
          right_goal = FollowJointTrajectoryGoal()
 
          right_goal.trajectory = JointTrajectory()
          right_goal.trajectory.joint_names = RIGHT_JOINT_NAMES
          
          right_goal.trajectory.points.append(JointTrajectoryPoint(positions=[0]*6, velocities=[0]*6,time_from_start=rospy.Duration(0.0)))
          right_goal.trajectory.points.append(JointTrajectoryPoint(positions=[0.5,0,-0.5,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(10.0)))
          right_goal.trajectory.points.append(JointTrajectoryPoint(positions=[1,0,-1,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(20.0)))
          right_goal.trajectory.points.append(JointTrajectoryPoint(positions=[1.57,0,-1.57,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(30.0)))
 
          return right_goal
 
def left_plan() :
 
          left_goal = FollowJointTrajectoryGoal()
          
          left_goal.trajectory = JointTrajectory()
          left_goal.trajectory.joint_names = LEFT_JOINT_NAMES
 
          left_goal.trajectory.points.append(JointTrajectoryPoint(positions=[0]*6, velocities=[0]*6,time_from_start=rospy.Duration(0.0)))
          left_goal.trajectory.points.append(JointTrajectoryPoint(positions=[0.5,0,-0.5,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(10.0)))
          left_goal.trajectory.points.append(JointTrajectoryPoint(positions=[1,0,-1,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(20.0)))
          left_goal.trajectory.points.append(JointTrajectoryPoint(positions=[1.57,0,-1.57,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(30.0)))
          
          return left_goal
 
def right_pub():
          right_client.send_goal(right_goal)
          right_client.wait_for_result()
 
def left_pub():
          left_client.send_goal(left_goal)
          left_client.wait_for_result()
 
def main():
          global left_client, right_client
          rospy.init_node("pub_dual_arm_move")
 
          left_client = actionlib.SimpleActionClient('left_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
          right_client = actionlib.SimpleActionClient('right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
          
          print("----------Preparing to connect----------")
          print("Waiting for server...")
          if left_client.wait_for_server():
                    print("Connect to left server")
          else:
                    sys.exit()
          if right_client.wait_for_server():
                    print("Connect to left server")
          else:
                    sys.exit()
 
          print(" ")
          print("------------Preparing to plan------------")
          print("Waitting for planning...")
          left_goal = left_plan()
          right_goal = right_plan()
          print("Planning success")
 
          print(" ")
          print("------------Preparing to pub------------")
 
          left_thread = threading.Thread(target=left_pub,args=(left_goal,))
          right_thread = threading.Thread(target=right_pub,args=(right_goal,))
          left_thread.start()
          right_thread.start()
          print("Pub success")
 
if __name__ == "__main__":
          main()
