#!/usr/bin/env python

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseArray


def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefacelistener(object):
  def __init__(self):
    super(MoveGroupPythonIntefacelistener, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('franka_moveit_trajectory',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    subs = rospy.Subscriber('waypoints', PoseArray, self.waypoint_callback)
    self.new_wps = []
    self.got_waypoints = False

  def waypoint_callback(self, data):

 
    self.new_wps = data.poses

    # print self.new_wps.poses

    self.got_waypoints = True


  def plan_cartesian_path(self, wps):
    group = self.group

    waypoints = []

    wpose = group.get_current_pose().pose

    print wpose

    # waypoints.append(copy.deepcopy(wpose))

    print "No of waypoints = " 
    print len(wps)

    for pose in wps:
      q = pose.orientation
      ql = [q.x, q.y, q.y, q.w]
      if all(x == 0 for x in ql):
        pose.orientation = wpose.orientation
        print "reusing original orientation"
      waypoints.append(copy.deepcopy(pose))


    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)


  def execute_plan(self, plan):
    group = self.group

    group.execute(plan, wait=True)


  def go_to_joint_state(self, state):
    group = self.group

    # joint_goal = group.get_current_joint_values()

    # joint_goal = [1.625171988935633, 1.4671819957759578, -2.342340774758504, -2.583995338835481, 1.7583991118566964, 1.2244194338367902, -0.6999209918781304]

    joint_goal = state

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)



def main():

  try:


    listener = MoveGroupPythonIntefacelistener()

    listener.group.set_max_velocity_scaling_factor(0.1)

    start_position = [1.9099858405502101, 1.21617677424499, -2.3731419994505454, -2.5192539586301104, 1.4097576812937405, 1.570779204079924, -0.6127737293200244]
    print(listener.go_to_joint_state(start_position))

    # while not rospy.is_shutdown():
        
    #     # print listener.group.get_current_pose().pose.orientation

    #     try:
    #         if listener.got_waypoints:

    #             listener.got_waypoints = False

    #             print ("\n\n\n\n\n")
    #             print listener.group.get_current_pose().pose.orientation

    #             wps = copy.deepcopy(listener.new_wps)
    #             cartesian_plan, fraction = listener.plan_cartesian_path(wps)
    #             print("Fraction: ", fraction)
    #             print("Cartesian plan:")
    #             # print(cartesian_plan)
    #             resp = 'y'
    #             while 'y' in resp or 'Y' in resp:
    #                 listener.display_trajectory(cartesian_plan)
    #                 resp = raw_input("Display again? Y/n \n")
                
    #             resp = 'n'
    #             resp = raw_input("Proceed with plan? y/N \n")
    #             if 'y' in resp or 'Y' in resp:
    #                 listener.execute_plan(cartesian_plan)
    #             else:
    #                 print("Aborting plan...")
    #             listener.new_wps = []

    #         time.sleep(1)
    #     except rospy.ROSInterruptException:
    #       return
    #     except KeyboardInterrupt:
    #       return
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()

