#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
import actionlib
import random

import franka_gripper.msg
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from koopman_pivot.msg import PivotObservation
from controller_manager_msgs.srv import *



class koopman_pol:

  def __init__(self, koopman_pol):
    self.acc_pub = rospy.Publisher("/panda/kth_joint_acceleration_effort_interface_controller/panda/equilibrium_pose",PoseStamped, queue_size=0 )
    self.joint_pose_pub = rospy.Publisher("/panda/kth_joint_pose_effort_interface_controller/panda/equilibrium_pose",PoseStamped, queue_size=0 )
    
    self.gripper_client = actionlib.SimpleActionClient('/panda/franka_gripper/move', franka_gripper.msg.MoveAction)
    self.gripper_client.wait_for_server()
    print("connected to grasping server")
    #controller services
    rospy.wait_for_service("/panda/controller_manager/load_controller")    
    self.s_load_controler = rospy.ServiceProxy('/panda/controller_manager/load_controller', LoadController)

    name="kth_joint_acceleration_effort_interface_controller"
    resp = self.s_load_controler.call(LoadControllerRequest(name))
    if resp.ok:
        print("Loaded", name)
    else:
        print("Error when loading", name)

    name="kth_joint_pose_effort_interface_controller"
    resp = self.s_load_controler.call(LoadControllerRequest(name))
    if resp.ok:
        print("Loaded", name)
    else:
        print("Error when loading", name)

    rospy.wait_for_service('/panda/controller_manager/switch_controller')
    self.s_switch_controler = rospy.ServiceProxy('/panda/controller_manager/switch_controller', SwitchController)
    

    self.pivot_sub = rospy.Subscriber("/pivot_observations",PivotObservation,self.pivot_callback)
    self.joint_state_sub = rospy.Subscriber("/pivot_observations",PivotObservation,self.pivot_callback)

    self.do_pivoting=False

    self.koopman_polecy = koopman_pol

    self.acc_old=0;
    self.finger_distance_old=0.2

  def pivot_callback(self,data):
    #print(data)
    if self.do_pivoting:
      rel_angle=data.rel_angle
      pole_ang_vel=data.pole_ang_vel
      grip_joint_pos=data.grip_joint_pos
      grip_joint_vel=data.grip_joint_vel
      finger_distance=data.finger_distance

      obs=1



      acc_signal, gripper_signal = self.get_koopman_action(obs)

      if not self.acc_old ==acc_signal:

        #puplish the acc with the posestamp message type
        acc = PoseStamped()
        acc.pose.position.x = 0
        acc.pose.position.y = 0
        acc.pose.position.z = 0
        acc.pose.orientation.x = 0
        acc.pose.orientation.y = 0
        acc.pose.orientation.z = acc_signal
        acc.pose.orientation.w = 0
        #only puplish if you have a new acc?
        self.acc_pub.publish(acc)
        self.acc_old=acc_signal

      if not self.finger_distance_old ==gripper_signal:
          #move the gripper
        #this waits for the gripper to be done before moving on (slow!)
        self.gripper_move_no_init(self.gripper_client,width=gripper_signal, speed=0.8)
        self.finger_distance_old=gripper_signal

  def swap_controller(self, name_start, name_stop):
    #do the swithcing
    rospy.wait_for_service('/panda/controller_manager/switch_controller')
    s_switch_controler = rospy.ServiceProxy('/panda/controller_manager/switch_controller', SwitchController)
    resp = s_switch_controler.call(name_start, name_stop, 1, False ,0.0)
    print(resp)
    if resp.ok == 1:
        print ("Started " + name_start + " successfully")
        print ("Stopped " + name_stop + " successfully")
    else:
        print ("Error when  " + name_start + " starting")
        print ("Error when  " + name_stop + " stopping")


  def stop_robot(self):
    #puplish the acc with the posestamp message type
    acc = PoseStamped()
    acc.pose.position.x = 0
    acc.pose.position.y = 0
    acc.pose.position.z = 0
    acc.pose.orientation.x = 0
    acc.pose.orientation.y = 0
    acc.pose.orientation.z = 0
    acc.pose.orientation.w = 0
    self.acc_pub.publish(acc)



  def goto_init_joint_pose(self, pose):
    self.joint_pose_pub.publish(pose)




  def gripper_move_no_init(self, client, width=0, speed=1.):
    """
    Use en establisht client to send goals
    :param width: Width (m) possition to move
    :param speed: Move velocity (m/s)
    :return: Bool success
    """
    client.send_goal(
            franka_gripper.msg.MoveGoal(
                width,
                speed
            )
        )
    return client.wait_for_result() 

  def open_gripper(self, width):
    self.gripper_move_no_init(self.gripper_client, width, 0.5)

  def close_gripper(self, width):
    self.gripper_move_no_init(self.gripper_client, width, 0.5)

  def get_koopman_action(self, obs):
  # here we feed in the rel_angle into the koopman polecy and get the next acction as acc and gripper pos

    #dummy action 
    acc=0 + random.uniform(-1.1, 1.1)*random.randint(0,1)
    finger_distance=0.01+ random.uniform(-0.01, 0.01)*random.randint(0,1)

    return acc, finger_distance




def print_commands():
  print("-------------------------")
  print("init - go to start pose")
  print("go - start the pivoting")
  print("og - open gripper")
  print("cg - close gripper")
  print("stop - stop robot")
  print("com - plot commands")
  print("-------------------------")




def main(args):
  rospy.init_node('koopman_pol', anonymous=True)
  print("starting")
  pivot_polecy_loc="adasda"
  width_open=0.04
  width_close=0.02
  #init joint pose is pose stamped bastardised
  init_joint_pose = PoseStamped()
  init_joint_pose.pose.position.x = 0.0012936384335864304
  init_joint_pose.pose.position.y = -0.0028746315517418353
  init_joint_pose.pose.position.z = -0.0013661144496737165
  init_joint_pose.pose.orientation.x = -1.5758416019746022
  init_joint_pose.pose.orientation.y = 1.5599549435240219
  init_joint_pose.pose.orientation.z = 1.6064429333877515
  init_joint_pose.pose.orientation.w = 0.7866897729630908

  kp = koopman_pol(pivot_polecy_loc)  
  print_commands()
  try:
    while not (rospy.is_shutdown()):
            r_str = raw_input()
            if ( r_str== '\x03'):           
                break

            if (r_str=='go'):
                print("*** Pivoting ***")
                try:
                  switch_controller = rospy.ServiceProxy('/panda/controller_manager/switch_controller', SwitchController)
                  ret = switch_controller(['kth_joint_acceleration_effort_interface_controller'], 
                    ['kth_joint_pose_effort_interface_controller'], 2, False, 0.0)
                except rospy.ServiceException, e:
                  print ("Service call failed: %s"%e)
                rospy.sleep(1)
                kp.do_pivoting=True

            if (r_str=='init'):
                print("*** Init robot ***")
                #prep robot                
                try:
                  switch_controller = rospy.ServiceProxy('/panda/controller_manager/switch_controller', SwitchController)
                  ret = switch_controller(['kth_joint_pose_effort_interface_controller'], 
                                           ['kth_joint_acceleration_effort_interface_controller'], 2, False, 0.0)
                except rospy.ServiceException, e:
                  print ("Service call failed: %s"%e)


                rospy.sleep(1)
                kp.goto_init_joint_pose(init_joint_pose)

            elif (r_str=='og'):
                print("*** Opening gripper ***")
                kp.open_gripper(width_open)

            elif (r_str=='cg'):
                print("*** closing gripper ***")
                kp.close_gripper(width_close)

            elif (r_str=='stop'):
                print("*** stopping ***")
                kp.do_pivoting=False
                kp.stop_robot()

            elif (r_str=='com'):
              print_commands()


  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)