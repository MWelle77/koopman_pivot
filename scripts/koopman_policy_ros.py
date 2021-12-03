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


import zmq
from time import sleep



MAX_ACC_FOR_ROBOT=18.0

class koopman_pol:

  def __init__(self, ):
        
    #koopman stuf
    print("Connecting to koopman server...")
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:50165")
    self.socket=socket
    self.obs=None
    self.gribber_joint_vel_threshold=2.5

    self.succes_count=0
    


    #Ros stuff
    self.acc_pub = rospy.Publisher("/kth_joint_acceleration_effort_interface_controller/panda/equilibrium_pose",PoseStamped, queue_size=1 )
    self.joint_pose_pub = rospy.Publisher("/kth_joint_pose_effort_interface_controller/panda/equilibrium_pose",PoseStamped, queue_size=1 )
    self.target_angle_pub = rospy.Publisher("/target_angle",Float64, queue_size=1 )
    self.stop_pub = rospy.Publisher("/kth_joint_acceleration_effort_interface_controller/panda/stop",PoseStamped, queue_size=1 )
    
    
    self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
    self.gripper_client.wait_for_server()
    self.gripper_client_force = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
    self.gripper_client_force.wait_for_server()
    self.gripper_client_stop = actionlib.SimpleActionClient('/franka_gripper/stop', franka_gripper.msg.StopAction)
    self.gripper_client_stop.wait_for_server()
    print("connected to grasping server")
   
    #controller services
    rospy.wait_for_service("/controller_manager/load_controller")    
    self.s_load_controler = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)

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

    rospy.wait_for_service('/controller_manager/switch_controller')
    self.s_switch_controler = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    

    self.pivot_sub = rospy.Subscriber("/pivot_observations",PivotObservation,self.pivot_callback)
    
    self.do_pivoting=False


    self.acc_old=0;
    self.finger_distance_old=0.02

    #sequence
    self.pivot_angle_seqence=[0, -45, 0, 60, -30, -5, 0]
    self.pivot_sequence_idx=0
    self.pivot_angle_succes_tol=np.deg2rad(3)
    self.do_pivoting_sequence=False

  def koopman_comunications(self, socket, obs):  
    socket.send(bytes(np.array2string(obs)))
    #sleep(0.01)
    #  Get the reply.
    message = socket.recv()
    actions=message.decode("utf-8")
    actions = actions.replace('[','')
    actions = actions.replace(']','')
    actions = np.fromstring(actions, dtype=float, sep=" ")

    return actions[0],actions[1]

  def update_koopman_actions(self):

    #print(self.obs)
    if (self.do_pivoting or self.do_pivoting_sequence) and not (self.obs==None).any():


            
      #rel_angle=data_msg.rel_angle
      #pole_ang_vel=data_msg.pole_ang_vel
      #grip_joint_pos=data_msg.grip_joint_pos
      #grip_joint_vel=data_msg.grip_joint_vel
      #finger_distance=data_msg.finger_distance
      obs=self.obs
      rel_angle=obs[0]

      #terminal condition for sequence
      if self.do_pivoting_sequence and np.abs(rel_angle)<self.pivot_angle_succes_tol and self.pivot_sequence_idx==len(self.pivot_angle_seqence)-1:
        print("succes ALL!")
        self.do_pivoting_sequence=False
        self.stop_robot()

      #advance sequence
      if self.do_pivoting_sequence and np.abs(rel_angle)<self.pivot_angle_succes_tol:
        print("succes part ", self.pivot_sequence_idx)
        #self.stop_robot() #fair?
        self.pivot_sequence_idx+=1
        self.target_angle_pub.publish(self.pivot_angle_seqence[self.pivot_sequence_idx])
        obs[0]=obs[0]+self.pivot_angle_seqence[self.pivot_sequence_idx]

      #print("obs:", obs)  
      #obs[4]= self.finger_distance_old

      # obs[0]*=-1
      # obs[1]*=-1
      # obs[2]*=-1
      # obs[3]*=-1

      print("obs: ", obs)

      acc_signal, gripper_signal = self.koopman_comunications(self.socket, obs)
      #acc_signal*=-1 
      
      #acc_signal=0
      #gripper_signal=0.02 #+ random.uniform(0.01,0.05)
      
      #gripper_signal=0.02


      #check if the robot joint is to fast and break
      # if np.abs(obs[3])> self.gribber_joint_vel_threshold:
      #   print(obs[3])
      #   acc_signal=0

      #acc_signal=0
      if gripper_signal>0.021:
         gripper_signal=0.021
      # else:
      #   gripper_signal=0.021
      #gripper_signal=0.023

      #if gripper_signal<0.0185:
       # gripper_signal=0.0185

      gripper_signal=np.round(gripper_signal,4)

      print("acc: ", acc_signal)
      print("gripper:", gripper_signal)

      #if not self.acc_old ==acc_signal:

      #puplish the acc with the posestamp message type
      acc = PoseStamped()
      acc.pose.position.x = 0
      acc.pose.position.y = 0
      acc.pose.position.z = 0
      acc.pose.orientation.x = 0
      acc.pose.orientation.y = 0
      sending_acc=np.clip(acc_signal,-MAX_ACC_FOR_ROBOT,MAX_ACC_FOR_ROBOT)
      #print("---------")
      #print(sending_acc)
      acc.pose.orientation.z = sending_acc
      acc.pose.orientation.w = 0
      #only puplish if you have a new acc?

      self.acc_pub.publish(acc)
      #self.acc_old=acc_signal

      if not np.round(self.finger_distance_old,4) == np.round(gripper_signal,4):
          #move the gripper
          self.gripper_stop(self.gripper_client_stop)
          self.gripper_move_no_init(self.gripper_client, gripper_signal, 0.7)
        #this waits for the gripper to be done before moving on (slow!)
        # if gripper_signal <=0.0195:
        #   self.gripper_grasp_no_init(self.gripper_client_force, gripper_signal, 0.7, force=0)
        # else:
        #   self.gripper_move_no_init(self.gripper_client, gripper_signal, 0.7)


          self.finger_distance_old=gripper_signal

      #terminal condition for single case
      if np.abs(rel_angle)<self.pivot_angle_succes_tol and self.do_pivoting:
        self.succes_count+=1
        print("+1")
        if self.succes_count > 50:
          print("***********************************************************************************succes!")

          self.do_pivoting=False
          self.stop_robot()
          self.gripper_move_no_init(self.gripper_client, 0.018, 0.7,)
      else:
        self.succes_count=0

      




  def pivot_callback(self,data):
    #print(data)
    self.obs=np.array([data.rel_angle, data.pole_ang_vel, data.grip_joint_pos, data.grip_joint_vel, data.finger_distance]).copy()
    
    #self.update_koopman_actions()
   

  def swap_controller(self, name_start, name_stop):
    #do the swithcing
    rospy.wait_for_service('/controller_manager/switch_controller')
    s_switch_controler = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
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
    self.stop_pub.publish(acc)



  def goto_init_joint_pose(self, pose):
    self.joint_pose_pub.publish(pose)
    self.gripper_move_no_init(self.gripper_client, 0.0205, 0.1)


  def gripper_stop(self, client):
    """
    Use en establisht client to send goals
    :param width: Width (m) possition to move
    :param speed: Move velocity (m/s)
    :return: Bool success
    """
    client.send_goal(
            franka_gripper.msg.StopGoal(
            )
        )
    return True#client.wait_for_result() 


  def gripper_grasp_no_init(self, client,  width=0, e_inner=0.1, e_outer=0.1, speed=0.1, force=1):
    """
    Wrapper around the franka_gripper/grasp action.
    http://docs.ros.org/kinetic/api/franka_gripper/html/action/Grasp.html
    :param width: Width (m) to grip
    :param e_inner: epsilon inner
    :param e_outer: epsilon outer
    :param speed: Move velocity (m/s)
    :param force: Force to apply (N)
    :return: Bool success
    """
    client.send_goal(
        franka_gripper.msg.GraspGoal(
            width,
            franka_gripper.msg.GraspEpsilon(e_inner, e_outer),
            speed,
            force
        )
    )
    return client.wait_for_result() 




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
    return True#client.wait_for_result() 

  def open_gripper(self, width):
    self.gripper_move_no_init(self.gripper_client, width, 0.5)

  def test_gripper(self, width):

    self.gripper_stop(self.gripper_client_stop)
    self.gripper_move_no_init(self.gripper_client, width, 0.01)


    # if width<=0.0191:
    #   self.gripper_grasp_no_init(self.gripper_client_force, width, 0.5, force=0)
    # else:
    #   self.gripper_move_no_init(self.gripper_client, width, 0.5)

    

  def close_gripper(self, width):
    self.gripper_move_no_init(self.gripper_client, width, 0.5)

  




def print_commands():
  print("-------------------------------------")
  print("init - go to start pose")
  print("go - start the pivoting")
  print("goall - performe sequence experiment")
  print("og - open gripper")
  print("cg - close gripper")
  print("stop - stop robot")
  print("tg - test gripper")
  print("com - plot commands")
  print("--------------------------------------")



  



def main(args):
  rospy.init_node('koopman_pol', anonymous=True)
  print("starting")
  
  

  width_open=0.04
  width_close=0.02
  #init joint pose is pose stamped bastardised

  init_joint_pose = PoseStamped()
  init_joint_pose.pose.position.x = 0.005395992080239873
  init_joint_pose.pose.position.y = -0.5604733576839663
  init_joint_pose.pose.position.z = -0.0005602394067845715
  init_joint_pose.pose.orientation.x = -2.13 #-2.197433645382279
  init_joint_pose.pose.orientation.y = 1.5643587700261006
  init_joint_pose.pose.orientation.z = 1.75   # middel
  #init_joint_pose.pose.orientation.z = 1.0094612440748357
  init_joint_pose.pose.orientation.w = 0.79

  kp = koopman_pol()  
  print_commands()
  readinput=True
  rate = rospy.Rate(1000)
  try:
    while not (rospy.is_shutdown()):      
      if readinput:
        r_str = raw_input()
      if ( r_str== '\x03'):           
          break

      if (r_str=='go'):
          print("*** Pivoting ***")
          try:
            switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            ret = switch_controller(['kth_joint_acceleration_effort_interface_controller'], 
              ['kth_joint_pose_effort_interface_controller'], 2, False, 0.0)
          except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
          rospy.sleep(1)
          kp.do_pivoting=True
          readinput=False
          r_str=''

      if (r_str=='goall'):
          print("*** Pivoting experimetn ***")
          try:
            switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            ret = switch_controller(['kth_joint_acceleration_effort_interface_controller'], 
              ['kth_joint_pose_effort_interface_controller'], 2, False, 0.0)
          except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
          rospy.sleep(1)
          print("*****starting pivot sequence:")
          print(kp.pivot_angle_seqence)
          print(kp.pivot_angle_succes_tol)
          kp.do_pivoting_sequence=True
          readinput=False
          r_str=''

      if (r_str=='init'):
          print("*** Init robot ***")
          #prep robot                
          try:
            switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            ret = switch_controller(['kth_joint_pose_effort_interface_controller'], 
                                     ['kth_joint_acceleration_effort_interface_controller'], 2, False, 0.0)
          except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)


          rospy.sleep(1)
          kp.goto_init_joint_pose(init_joint_pose)

     
          

      elif (r_str=='og'):
          print("*** Opening gripper ***")
          kp.open_gripper(width_open)

      elif (r_str=='cg'):
          print("*** closing gripper ***")
          kp.close_gripper(width_close)

      elif (r_str=='tg'):
          print("*** testing gripper ***")
          g_width = float(raw_input())
          kp.test_gripper(g_width)

      elif (r_str=='stop'):
          print("*** stopping ***")
          kp.do_pivoting=False
          kp.stop_robot()

      elif (r_str=='com'):
        print_commands()

      kp.update_koopman_actions()
      rate.sleep()
            

  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)