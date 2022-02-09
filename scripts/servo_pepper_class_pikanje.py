#! /usr/bin/env python3
import rospy

from sensor_msgs.msg import JointState
from franka_position_servo.msg import JointCommand, RobotState, EndPointState
from franka_msgs.msg import FrankaState, ErrorRecoveryAction, ErrorRecoveryActionGoal
from std_msgs.msg import Float64MultiArray, Bool, String, Int32, Header
from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped, TransformStamped, Point
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from controller_manager_msgs.srv import SwitchController

import numpy as np
from copy import deepcopy
import quaternion

import moveit_commander

import time

import tf2_ros
from geometry_msgs.msg import Vector3

import actionlib
import franka_gripper.msg
from std_srvs.srv import SetBool


names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']


import tf.transformations as tr

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Vector3


def pose_to_pq(msg):
    """Convert a C{geometry_msgs/Pose} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.position.x, msg.position.y, msg.position.z])
    q = np.array([msg.orientation.x, msg.orientation.y,
                  msg.orientation.z, msg.orientation.w])
    return p, q

def pose_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/PoseStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return pose_to_pq(msg.pose)

def msg_to_se3(msg):
    """Conversion from geometric ROS messages into SE(3)

    @param msg: Message to transform. Acceptable types - C{geometry_msgs/Pose}, C{geometry_msgs/PoseStamped},
    C{geometry_msgs/Transform}, or C{geometry_msgs/TransformStamped}
    @return: a 4x4 SE(3) matrix as a numpy array
    @note: Throws TypeError if we receive an incorrect type.
    """
    if isinstance(msg, Pose):
        p, q = pose_to_pq(msg)
    elif isinstance(msg, PoseStamped):
        p, q = pose_stamped_to_pq(msg)
    elif isinstance(msg, Transform):
        p, q = transform_to_pq(msg)
    elif isinstance(msg, TransformStamped):
        p, q = transform_stamped_to_pq(msg)
    else:
        raise TypeError("Invalid type for conversion to SE(3)")
    norm = np.linalg.norm(q)
    if np.abs(norm - 1.0) > 1e-3:
        raise ValueError(
            "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                str(q), np.linalg.norm(q)))
    elif np.abs(norm - 1.0) > 1e-6:
        q = q / norm
    g = tr.quaternion_matrix(q)
    g[0:3, -1] = p
    return g

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True






class FrankaControl():

    def __init__(self):

        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2listener = tf2_ros.TransformListener(self.tf2Buffer)


        self.franka_fault = None

        self.pub = rospy.Publisher('/motion_controller/arm/joint_commands',JointCommand, queue_size = 1, tcp_nodelay = True)
        self.error_recovery_pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=1)

        self.pose_pub = rospy.Publisher('/franka_state_controller/link8_pose', PoseStamped, queue_size=1)

        self.opto_tf_pub = rospy.Publisher('/force_global_tf', Float64MultiArray, queue_size=1)

        # self.pose_reached_pub = rospy.Publisher('/position_reached', Bool, queue_size=1)

        self.control_mode_pub = rospy.Publisher('/state_machine', String, queue_size=1)

        rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/franka_state_controller/robot_state', RobotState, self.jacobian_callback)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.franka_state_callback)

        rospy.Subscriber('/moveit/arm_position_controller/command', Float64MultiArray, self.servo_callback_moveit)

        rospy.Subscriber('/state_machine', String, self.state_machine_callback)

        self.impedance_flag = False

        self.rate_hz = 100.
        self.rate = rospy.Rate(self.rate_hz)

        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        self.joint_pos = np.asarray([])

        self.cartesian_pose = None
        self.cart_pose_trans_mat = None

        self.jacobian = None
        self.new_servo_ref = False
        self.new_servo_ref_moveit = False

        self.sm_state = "idle"

        self.motion_success = True

        while not rospy.is_shutdown() and len(self.joint_pos) != 7:
            continue

        # ret = self.switch_controller(['position_joint_trajectory_controller'], ['joint_position_controller'], 2, True, 1.0)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        # self.group.set_named_target('ready')
        # self.group.go()

        # self.go_to_joint_state(self.group, neutral_pose)
        # ret = self.switch_controller(['joint_position_controller'], ['position_joint_trajectory_controller'], 2, True, 1.0)

        self.gripper_grasp_flag_ = False

        self.control_mode = "idle"

    def state_machine_callback(self, msg):

        self.sm_state = msg.data

        if "grasping" in self.sm_state:
            self.gripper_grasp_flag_ = True

        if "impedance" in self.sm_state:
            self.impedance_flag = True
        else:
            self.impedance_flag = False



    def franka_state_callback(self, msg):
        self.cart_pose_trans_mat = np.asarray(msg.O_T_EE).reshape(4,4,order='F')
        self.cartesian_pose = {
            'position': self.cart_pose_trans_mat[:3,3],
            'orientation': quaternion.from_rotation_matrix(self.cart_pose_trans_mat[:3,:3]) }
        self.franka_fault = self.franka_fault or msg.last_motion_errors.communication_constraints_violation or msg.last_motion_errors.cartesian_reflex
        

    def jacobian_callback(self, msg):
        self.jacobian = np.asarray(msg.O_Jac_EE).reshape(6,7,order = 'F')


    def joint_state_callback(self, msg):

        temp_joint_pos = []
        for n in names:
            idx = msg.name.index(n)
            temp_joint_pos.append(msg.position[idx])

        self.joint_pos = np.asarray(deepcopy(temp_joint_pos))


    def servo_callback_moveit(self, msg):

        self.servo_ref = np.asarray(msg.data)
        if len(self.servo_ref) < 7:
            print("got wrong servo ref??")
        self.new_servo_ref_moveit = True
        # print("ref from moveit")



    def global_displacement_to_q(self, dp_ref):

        j = self.jacobian[0:3, :]
        j_inv = np.linalg.pinv(j)
        return np.dot(j_inv, dp_ref)


    def reset_communication_error(self):
        err_rec_msg = ErrorRecoveryActionGoal()
        if self.franka_fault:
            self.error_recovery_pub.publish(err_rec_msg)
            # time.sleep(1./self.rate_hz)
            self.franka_fault = False



    def go_to_joint_state(self):

        joint_goal = [i for i in self.servo_ref]
        # print("in joint goal")
        # print(joint_goal)
        a = self.group.go(joint_goal, wait=True)
        b = self.group.stop()
        
        print("group go success: ", a)
        
        current_joints = self.group.get_current_joint_values()
        return (a and all_close(joint_goal, current_joints, 0.01))



if __name__ == '__main__':
    

    rospy.init_node("franka_pepper_picking_node")

    rospy.wait_for_service('/controller_manager/list_controllers')

    ctrl = FrankaControl()

    pubmsg = JointCommand()
    pubmsg.names = names # names of joints (has to be 7 and in the same order as the command fields (positions, velocities, efforts))
    
    exit_stuck = 0

    gripping_ = False

    if gripping_:
        client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
        print("wait for gripper server...")
        client.wait_for_server()
        goal = franka_gripper.msg.MoveActionGoal()
        goal.goal.width = 0.0
        goal.goal.speed = 1.0

    # print("wait for position reached...")

    # rospy.wait_for_service('position_reached')
    # pos_reached = rospy.ServiceProxy('position_reached', SetBool)

    print("starting...")

    p = PoseStamped()
    
    
    p.header.frame_id = "panda_link0"
    p.pose.position.x = 0
    p.pose.position.y = 0.7
    p.pose.position.z = 0.0
    box_name = "table"
    box_size = (0.6,0.4,0.7)
    ctrl.scene.add_box(box_name, p, box_size)



    # box_is_known = False
    # while not box_is_known:
    #     print("table is still not here..")
    #     ctrl.scene.add_box(box_name, p, box_size)
    #     time.sleep(.1)
    #     box_is_known = box_name in ctrl.scene.get_known_object_names()
    # else:
    #     print("Table added as obstacle into the scene")


    while not rospy.is_shutdown():

        ctrl.pose_pub.publish(ctrl.group.get_current_pose())
    
        try:
            trans = ctrl.tf2Buffer.lookup_transform('panda_link0', 'optoforce', rospy.Time())
            # print(trans)

            msgPose = PoseStamped()
            msgPose.pose.position.x = trans.transform.translation.x
            msgPose.pose.position.y = trans.transform.translation.y
            msgPose.pose.position.z = trans.transform.translation.z

            msgPose.pose.orientation.x = trans.transform.rotation.x
            msgPose.pose.orientation.y = trans.transform.rotation.y
            msgPose.pose.orientation.z = trans.transform.rotation.z
            msgPose.pose.orientation.w = trans.transform.rotation.w

            msgPose.header.stamp = rospy.Time.now()
            
            optoforce_tf_matrix = msg_to_se3(msgPose)

            tf_list = optoforce_tf_matrix.flatten()

            msg_tf = Float64MultiArray()
            msg_tf.data = tf_list

            ctrl.opto_tf_pub.publish(msg_tf)

            # ctrl.pose_pub.publish(msgPose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("no transform")
            continue
        # msg = geometry_msgs.msg.Twist()


        if gripping_:
            if ctrl.gripper_grasp_flag_:
                print("gripper grasping!!!")
                client.send_goal(goal.goal)
                client.wait_for_result()
                print(client.get_result())


            # resp1 = pos_reached(True)
            ctrl.gripper_grasp_flag_ = False

        if ctrl.new_servo_ref or ctrl.new_servo_ref_moveit:

            try:
                plan_move = any(abs(ctrl.servo_ref-ctrl.joint_pos)>0.1)
            except ValueError:
                continue
            if ctrl.motion_success and plan_move and not ctrl.impedance_flag:
                msg_switch = SwitchController()
                msg_switch.start_controllers = 'position_joint_trajectory_controller'
                msg_switch.stop_controllers = 'joint_position_controller'
                msg_switch.strictness = 2
                msg_switch.start_asap = True
                msg_switch.timeout = 1.0
                # ret = ctrl.switch_controller(['position_joint_trajectory_controller', 'joint_position_controller', '2', 'True', '1.0'])
                ret = ctrl.switch_controller(['position_joint_trajectory_controller'], ['joint_position_controller'], 2, True, 1.0)
                # ret = ctrl.switch_controller.call(msg_switch)
                ctrl.reset_communication_error()

                ctrl.motion_success = ctrl.go_to_joint_state()

                msg_switch = SwitchController()
                msg_switch.start_controllers = 'joint_position_controller'
                msg_switch.stop_controllers = 'position_joint_trajectory_controller'
                msg_switch.strictness = 2
                msg_switch.start_asap = True
                msg_switch.timeout = 1.0
                # ret = ctrl.switch_controller(['position_joint_trajectory_controller', 'joint_position_controller', '2', 'True', '1.0'])
                # ret = ctrl.switch_controller(msg_switch)
                ret = ctrl.switch_controller(['joint_position_controller'], ['position_joint_trajectory_controller'], 2, True, 1.0)
            else:
                dist = np.linalg.norm(ctrl.joint_pos - ctrl.servo_ref)
                if (not ctrl.motion_success) and dist > 0.1:
                    pubmsg.position = list(ctrl.joint_pos + 0.1 * (ctrl.servo_ref - ctrl.joint_pos))
                    exit_stuck += 1
                else:
                    pubmsg.position = ctrl.servo_ref
                pubmsg.mode = pubmsg.POSITION_MODE 

                ctrl.reset_communication_error()
                ctrl.pub.publish(pubmsg)

            if exit_stuck > 100:
                if not ctrl.impedance_flag:
                    ctrl.motion_success = True
                exit_stuck = 0


            reached_ref = all(abs(ctrl.servo_ref-ctrl.joint_pos)<0.01)
            if reached_ref:
                ctrl.servo_ref = []
                ctrl.new_servo_ref = False
                ctrl.new_servo_ref_moveit = False
                ctrl.motion_success = True

        ctrl.rate.sleep()


