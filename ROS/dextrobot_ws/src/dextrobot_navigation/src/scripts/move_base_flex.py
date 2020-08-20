#!/usr/bin/env python
#
# @author Jorge Santos
# License: 3-Clause BSD

import actionlib
import copy

import rospy
import nav_msgs.srv as nav_srvs
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped
from move_base.cfg import MoveBaseConfig
from geometry_msgs.msg import Twist

import numpy as np

"""
move_base legacy relay node:
Relays old move_base actions to the new mbf move_base action, similar but with richer result and feedback.
We also relay the simple goal topic published by RViz, the make_plan service and dynamic reconfiguration
calls (note that some parameters have changed names; see http://wiki.ros.org/move_base_flex for details)
"""

distance_to_goal = 0

def simple_goal_cb(msg):
    mbf_cmb_ac.send_goal(mb_msgs.MoveBaseGoal(target_pose=msg))
    #mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg, planner="WaypointGlobalPlanner"))
    #rospy.logdebug("Relaying move_base_simple/goal pose to mbf")

def getPathLength(path_msg, start_pose_index, target_pose_index):
    if target_pose_index <= start_pose_index:
       return 0

    if start_pose_index < 0:
       start_pose_index = 0

    if target_pose_index < 0 or target_pose_index > len(path_msg.poses) - 1:
       target_pose_index = len(path_msg.poses) - 1
    
    path_length = 0
    for i in range(start_pose_index, target_pose_index-1):
        position_a_x = path_msg.poses[i].pose.position.x
        position_b_x = path_msg.poses[i+1].pose.position.x
        position_a_y = path_msg.poses[i].pose.position.y
        position_b_y = path_msg.poses[i+1].pose.position.y
        path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2)) 
    return path_length  


def mb_execute_cb(msg):
    
    max_replanning_attempts = 1
    replanning_counter = 0

    last_executed_lane_plan_length = 10000000
    min_new_plan_lengh_difference = 3 # safety distance used to avoid loops in switching between planners
    
    is_following_lane = True
    
    rospy.loginfo("New goal received by move base flex script")
       
    while True:
        # Stop if a new goal is received or the goal is cancelled
        if mb_as.is_new_goal_available() or mb_as.is_preempt_requested():
            mbf_ep_ac.cancel_all_goals() 
            velocity_publisher.publish(Twist())      
            mb_as.set_preempted()  
            return

        # Try to follow the lane
        mbf_gp_ac.send_goal(mbf_msgs.GetPathGoal(target_pose=msg.target_pose, use_start_pose = False, planner="WaypointGlobalPlanner", concurrency_slot = 1))
        mbf_gp_ac.wait_for_result()

        planning_result = mbf_gp_ac.get_result()
        
        new_plan_length = getPathLength(planning_result.path, 0, len(planning_result.path.poses)-1)        
                
        loop_detected = (not is_following_lane) and (new_plan_length > last_executed_lane_plan_length - min_new_plan_lengh_difference)

        if planning_result.outcome != mbf_msgs.GetPathResult.SUCCESS or loop_detected:
            is_following_lane = False

            # Try using SPBL to go to some of the following points on the lane
            # Get the remaining path                  
            mbf_gp_ac.send_goal(mbf_msgs.GetPathGoal(target_pose=msg.target_pose, use_start_pose = False, planner="NoObstaclesWaypointGlobalPlanner", concurrency_slot = 1))
            mbf_gp_ac.wait_for_result()            
            
            short_plan_found = False
            planning_result = mbf_gp_ac.get_result()
            if planning_result.outcome == mbf_msgs.GetPathResult.SUCCESS:
               path_poses = planning_result.path.poses
               lane_path_length = getPathLength(planning_result.path, 0, len(planning_result.path.poses)-1)   
               number_of_remaining_waypoints = len(path_poses)
               step_size = 10 # in number of waypoints
               first_waypoint_index = max(0,min(number_of_remaining_waypoints-1, 15))
               last_waypoint_index = max(0,min(number_of_remaining_waypoints-1, 80))
               for waypoint_index in range(first_waypoint_index, last_waypoint_index, step_size ):
                  mbf_gp_ac.send_goal(mbf_msgs.GetPathGoal(target_pose=path_poses[waypoint_index], planner="SBPLLatticePlanner", concurrency_slot = 2))
                  while not mbf_gp_ac.wait_for_result(rospy.Duration.from_sec(0.1)):
                      if mb_as.is_new_goal_available() or mb_as.is_preempt_requested():
                          mbf_ep_ac.cancel_all_goals()
                          mbf_gp_ac.cancel_all_goals()  
                          velocity_publisher.publish(Twist())      
                          mb_as.set_preempted()  
                          return     
                  planning_result = mbf_gp_ac.get_result()
                  if planning_result.outcome == mbf_msgs.GetPathResult.SUCCESS:
                      free_path_length = getPathLength(planning_result.path, 0, len(planning_result.path.poses)-1)  
                      if free_path_length > lane_path_length * 2:
                          # the robot would probably turn around the corridor, try the next waypoint
                          continue    
                      short_plan_found = True  
                      # Append the reamaining part of the plan
                      if waypoint_index < len(path_poses) - 1:
                          planning_result.path.poses += path_poses[waypoint_index:]
                      break        
            
            # Try to use SBPL to go to the goal
            if not short_plan_found:
                mbf_gp_ac.send_goal(mbf_msgs.GetPathGoal(target_pose=msg.target_pose, planner="SBPLLatticePlanner", concurrency_slot = 4))
                while not mbf_gp_ac.wait_for_result(rospy.Duration.from_sec(0.1)):
                    if mb_as.is_new_goal_available() or mb_as.is_preempt_requested():
                        mbf_ep_ac.cancel_all_goals()
                        mbf_gp_ac.cancel_all_goals()  
                        velocity_publisher.publish(Twist())      
                        mb_as.set_preempted()  
                        return     
                planning_result = mbf_gp_ac.get_result()
        else:
           last_executed_lane_plan_length = new_plan_length 
           is_following_lane = True        
           
    
        if planning_result.outcome != mbf_msgs.GetPathResult.SUCCESS:
            # There is nothing left to try, abort  
            mbf_ep_ac.cancel_all_goals() 
            velocity_publisher.publish(Twist())
            mb_as.set_aborted(mb_msgs.MoveBaseResult(), planning_result.message) 
            return                        
        

        # Execute the path
        rospy.loginfo("Execute the plan")
        mbf_ep_ac.send_goal(mbf_msgs.ExePathGoal(path=planning_result.path, concurrency_slot = 3))

        # Wait a bit 
        execution_completed = mbf_ep_ac.wait_for_result(rospy.Duration.from_sec(0.1))

        # If the execution of the global plan is completed, return        
        if execution_completed:
            if mbf_ep_ac.get_result() == mbf_msgs.ExePathResult.SUCCESS:
                mb_as.set_succeeded(mb_msgs.MoveBaseResult(), "Goal reached.")                            
                return
            else:
                velocity_publisher.publish(Twist())      
                if replanning_counter >= max_replanning_attempts:
                  mb_as.set_aborted(mb_msgs.MoveBaseResult(), mbf_ep_ac.get_result().message)                     
                  return
                else:
                  replanning_counter+=1        
        else:
          replanning_counter = 0      
          # Otherwise, iterate again
      

def make_plan_cb(request):
    mbf_gp_ac.send_goal(mbf_msgs.GetPathGoal(start_pose=request.start, target_pose=request.goal,
                                             use_start_pose = bool(request.start.header.frame_id),
                                             tolerance=request.tolerance))
    rospy.logdebug("Relaying legacy make_plan service to mbf get_path action server")
    mbf_gp_ac.wait_for_result()

    status = mbf_gp_ac.get_state()
    result = mbf_gp_ac.get_result()

    rospy.logdebug("MBF get_path execution completed with result [%d]: %s", result.outcome, result.message)
    if result.outcome == mbf_msgs.GetPathResult.SUCCESS:
        return nav_srvs.GetPlanResponse(plan=result.path)

def mbf_feedback_cb(feedback):
    distance_to_goal = feedback.dist_to_goal

if __name__ == '__main__':
    rospy.init_node("move_base")

    # TODO what happens with malformed target goal???  FAILURE  or INVALID_POSE
    # txt must be:  "Aborting on goal because it was sent with an invalid quaternion"   

    # move_base_flex get_path and move_base action clients
    mbf_mb_ac = actionlib.SimpleActionClient("mbf_costmap_nav/move_base", mbf_msgs.MoveBaseAction)
    mbf_gp_ac = actionlib.SimpleActionClient("mbf_costmap_nav/get_path", mbf_msgs.GetPathAction)
    mbf_ep_ac = actionlib.SimpleActionClient("mbf_costmap_nav/exe_path", mbf_msgs.ExePathAction)
    mbf_mb_ac.wait_for_server(rospy.Duration(20))
    mbf_gp_ac.wait_for_server(rospy.Duration(10))
    mbf_ep_ac.wait_for_server(rospy.Duration(10))

    # move_base simple topic and action server
    mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)
    mb_as = actionlib.SimpleActionServer('combined_move_base', mb_msgs.MoveBaseAction, mb_execute_cb, auto_start=False)
    mb_as.start()

    mbf_cmb_ac = actionlib.SimpleActionClient("combined_move_base", mb_msgs.MoveBaseAction)

    # move_base make_plan service
    mb_mps = rospy.Service('~make_plan', nav_srvs.GetPlan, make_plan_cb)

    # publisher of the twist command for safery reasons
    velocity_publisher = rospy.Publisher('~cmd_vel', Twist, queue_size=1)        

    rospy.spin()
