#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from koopman_pivot.msg import PivotObservation
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError


# ARUCO_DICT = {
#   "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
#   "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
#   "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
#   "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
#   "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
#   "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
#   "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
#   "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
#   "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
#   "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
#   "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
#   "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
#   "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
#   "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
#   "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
#   "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
#   "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
#   #"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
#   #"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
#   #"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
#   #"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
# }



class pivot_observer:

  def __init__(self, target_angle):
    self.image_pub = rospy.Publisher("/pivot_inpainted",Image, queue_size=1)
    self.observation_pub = rospy.Publisher("/pivot_observations", PivotObservation, queue_size=1)

    self.msgPivotObservation=PivotObservation()

    self.pole_pos_old=(0,rospy.get_rostime())


    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.joint_state_sub = rospy.Subscriber("/joint_states",JointState,self.jointcallback)
    #self.aruco_type="DICT_4X4_50"
    self.target_angle_sub = rospy.Subscriber("/target_angle",Float64,self.targetcallback)
    self.target_angle=target_angle

    self.pole_vel_vec=list()
    self.pole_vel_window=5

    self.joint_offset=1.75


  def pub_observations(self):    
    self.observation_pub.publish(self.msgPivotObservation)

  def targetcallback(self, data):
    #print(data.data)
    self.target_angle=data.data


  def jointcallback(self, data):
    #read the joint XX postition + joint XX velocity
    #print(data)
    joint_idx=data.name.index("panda_joint6")
    pivot_joint_pos=data.position[joint_idx]-self.joint_offset
    pivot_joint_pos*=-1
    pivot_joint_vel=data.velocity[joint_idx]
    pivot_joint_vel*=-1
    gripper_idx=data.name.index("panda_finger_joint1")
    gripper_finger_pos=data.position[gripper_idx]
    self.msgPivotObservation.grip_joint_pos =pivot_joint_pos    
    self.msgPivotObservation.grip_joint_vel =pivot_joint_vel
    self.msgPivotObservation.finger_distance =0.02#gripper_finger_pos*2 #symetrical gripper


  def callback(self,data):
    try:
      #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #cut and rotate
    cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
    #cv_image =cv_image[400:1420,:,:]
    cv_image =cv_image[100:540,:,:]
    #print(cv_image.shape)

    cv_image =self.det_pivot_angle(cv_image, self.target_angle)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)


    

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)
    #dim=(int(cv_image.shape[0]*0.8), int(cv_image.shape[1]*0.8))
    

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    #self.angle_pub.publish(rel_angle)

  def det_pivot_angle(self, image, target_angle):

    #detect color blobs
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #morph kernel
    kernel = np.ones((5,5),np.uint8)
    #detect green
    mask_green = cv2.inRange(hsv, (40, 25, 25), (76, 255,255))
    mask_green=cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    _,contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blob_green = max(contours_green, key=lambda el: cv2.contourArea(el))
    M_green = cv2.moments(blob_green)
    center_green = (int(M_green["m10"] / M_green["m00"]), int(M_green["m01"] / M_green["m00"]))

    cv2.circle(image, center_green, 5, (255,0,0), -1)

    #cv2.imshow("Image green", mask_green)
    #cv2.waitKey(3)

    #detect yellow 
    mask_yellow = cv2.inRange(hsv, (25, 175, 50), (40, 255,255)) #evening
    #mask_yellow = cv2.inRange(hsv, (15, 160, 0), (30, 255,255)) #morning
    mask_yellow=cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    _,contours_yellow , _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blob_yellow  = max(contours_yellow , key=lambda el: cv2.contourArea(el))
    M_yellow  = cv2.moments(blob_yellow )
    center_yellow  = (int(M_yellow["m10"] / M_yellow["m00"]), int(M_yellow["m01"] / M_yellow["m00"]))

    cv2.circle(image, center_yellow, 5, (255,0,0), -1)

    #cv2.imshow("Image yellow", mask_yellow)
    #cv2.waitKey(3)

    #detect red
    #Mat1b mask1, mask2;
    #inRange(hsv, Scalar(0, 70, 50), Scalar(10, 255, 255), mask1);
    #inRange(hsv, Scalar(170, 70, 50), Scalar(180, 255, 255), mask2);

    #Mat1b mask = mask1 | mask2; 
    mask_red_1 = cv2.inRange(hsv, (0,70,50), (15,255,255))
    mask_red_2 = cv2.inRange(hsv, (170,70,50), (180,255,255))
    mask_red = cv2.bitwise_or(mask_red_1,mask_red_2)
    mask_red=cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    _,contours_red , _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blob_red  = max(contours_red , key=lambda el: cv2.contourArea(el))
    M_red = cv2.moments(blob_red )
    center_red  = (int(M_red["m10"] / M_red["m00"]), int(M_red["m01"] / M_red["m00"]))

    cv2.circle(image, center_red, 5, (255,0,0), -1)

    #cv2.imshow("Image red", mask_red)
    #cv2.waitKey(3)

    #draw lines
    image, line_pole = drawlinetwopoints(image, center_green, center_yellow, (0, 0, 255))

    #adjust for 90 deg offset
    px=float(center_red[0]-center_yellow[0])
    py=float(center_red[1]-center_yellow[1])
    #rotate with target angle
    tx=px*np.cos(np.deg2rad(90)) -py*np.sin(np.deg2rad(90))
    ty=px*np.sin(np.deg2rad(90)) + py*np.cos(np.deg2rad(90))
    px=int(tx+center_yellow[0])
    py=int(ty+center_yellow[1])

    center_red=(px,py)

    image, line_gripper = drawlinetwopoints(image, center_red, center_yellow, (0, 255, 0))
    #add target line
    px=float(center_red[0]-center_yellow[0])
    py=float(center_red[1]-center_yellow[1])
    #rotate with target angle
    tx=px*np.cos(np.deg2rad(target_angle)) -py*np.sin(np.deg2rad(target_angle))
    ty=px*np.sin(np.deg2rad(target_angle)) + py*np.cos(np.deg2rad(target_angle))
    px=int(tx+center_yellow[0])
    py=int(ty+center_yellow[1])
    image, line_target = drawlinetwopoints(image, center_yellow, (px, py), (255, 255, 0))

    #calculate angle
    tantheta=(line_pole[0]*line_gripper[1]-line_gripper[0]*line_pole[1])/(line_gripper[0]*line_pole[0]+line_gripper[1]*line_pole[1])
    rel_angle= np.arctan(tantheta)
    rel_angle= np.rad2deg(rel_angle) - target_angle 
    #inverse to make it coherent with sim 
    rel_angle*=-1

    #calculate the angular velocity of the pole
    vel_angle=(rel_angle-self.pole_pos_old[0])/(rospy.get_rostime()-self.pole_pos_old[1]).to_sec()

    #average vel to make more stable
    self.pole_vel_vec.append(vel_angle)
    if len(self.pole_vel_vec)> self.pole_vel_window:
      pole_vel_vec_window=self.pole_vel_vec.pop(0)
    else:
      pole_vel_vec_window=self.pole_vel_vec

    vel_angle_windowed=np.mean(pole_vel_vec_window)

    if not self.pole_pos_old[1] ==rospy.get_rostime():
      self.pole_pos_old=(rel_angle,rospy.get_rostime())


    #inpaing angle
    cv2.putText(image, str(np.round(rel_angle,2)) + " deg",
        (5, 30), cv2.FONT_HERSHEY_SIMPLEX,
        1, (200, 200, 200), 2)

    # cv2.putText(image, str(np.round(vel_angle_windowed,2)) + " deg/s",
    #     (5, 75), cv2.FONT_HERSHEY_SIMPLEX,
    #     1, (0, 255, 0), 2)

    #update observations
    if not np.isnan(rel_angle):
      self.msgPivotObservation.rel_angle = np.deg2rad(rel_angle)
    if not np.isnan(vel_angle_windowed):
      self.msgPivotObservation.pole_ang_vel = np.deg2rad(vel_angle_windowed)*0

    #print(rel_angle)
    if np.abs(rel_angle) <=3:
      cv2.circle(image, (455,25), 15, (0,255,0), -1)
    else:
      cv2.circle(image, (455,25), 15, (0,0,255), -1)




    return image
    
  # def det_pivot_angle(self, image, target_angle):

  #   rel_angle=np.NAN
  #   vel_angle_windowed=np.NAN

  #   # loop over the types of ArUco dictionaries
  #   #for (arucoName, arucoDict) in ARUCO_DICT.items():

  #   arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_type])
  #   arucoParams = cv2.aruco.DetectorParameters_create()
  #   (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

  #   # verify *at least* one ArUco marker was detected
  #   markers=[]
  #   if len(corners) > 0:
  #     # flatten the ArUco IDs list
  #     ids = ids.flatten()
  #     # loop over the detected ArUCo corners
  #     m_0=0
  #     m_1=0
  #     for (markerCorner, markerID) in zip(corners, ids):
  #       # extract the marker corners (which are always returned in
  #       # top-left, top-right, bottom-right, and bottom-left order)
  #       corners = markerCorner.reshape((4, 2))
  #       (topLeft, topRight, bottomRight, bottomLeft) = corners
  #       # convert each of the (x, y)-coordinate pairs to integers
  #       topRight = (int(topRight[0]), int(topRight[1]))
  #       bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
  #       bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
  #       topLeft = (int(topLeft[0]), int(topLeft[1]))

               

  #       # draw the bounding box of the ArUCo detection
  #       cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
  #       cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
  #       cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
  #       cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
  #       # compute and draw the center (x, y)-coordinates of the ArUco
  #       # marker
  #       cX = int((topLeft[0] + bottomRight[0]) / 2.0)
  #       cY = int((topLeft[1] + bottomRight[1]) / 2.0)
  #       cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
  #       # draw the ArUco marker ID on the image
  #       cv2.putText(image, str(markerID),
  #         (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
  #         0.5, (0, 255, 0), 2)
  #       #print("[INFO] ArUco marker ID: {}".format(markerID))

  #       markers.append((markerID, topRight, bottomRight, bottomLeft, topLeft))

  #     #now do the lines:
  #     line_pole=None
  #     line_gripper=None
  #     line_target=None
  #     for markerID, topRight, bottomRight, bottomLeft, topLeft in markers:

  #       if markerID==0:       

  #         #calculate middel points
  #         topMiddel = (int((topLeft[0]+topRight[0])/2.0), int((topLeft[1]+topRight[1])/2.0))
  #         cv2.circle(image, topMiddel, 4, (0, 0, 255), -1)
  #         bottomMiddel=(int((bottomLeft[0]+bottomRight[0])/2.0), int((bottomLeft[1]+bottomRight[1])/2.0))
  #         cv2.circle(image, bottomMiddel, 4, (0, 0, 255), -1)

  #         image, line_pole = drawlinetwopoints(image, topMiddel, bottomMiddel, (0, 0, 255))

  #       if markerID==1:       

  #         #calculate middel points
  #         topMiddel = (int((topLeft[0]+topRight[0])/2.0), int((topLeft[1]+topRight[1])/2.0))
  #         cv2.circle(image, topMiddel, 4, (0, 0, 255), -1)
  #         bottomMiddel=(int((bottomLeft[0]+bottomRight[0])/2.0), int((bottomLeft[1]+bottomRight[1])/2.0))
  #         cv2.circle(image, bottomMiddel, 4, (0, 0, 255), -1)

  #         image, line_gripper = drawlinetwopoints(image, topMiddel, bottomMiddel, (0, 255, 0))

  #         #add target line
  #         cX = int((topLeft[0] + bottomRight[0]) / 2.0)
  #         cY = int((topLeft[1] + bottomRight[1]) / 2.0)

  #         px=float(topMiddel[0]-cX)
  #         py=float(topMiddel[1]-cY)

  #         #rotate with target angle
  #         tx=px*np.cos(np.deg2rad(target_angle)) -py*np.sin(np.deg2rad(target_angle))
  #         ty=px*np.sin(np.deg2rad(target_angle)) + py*np.cos(np.deg2rad(target_angle))

  #         px=int(tx+cX)
  #         py=int(ty+cY)
  #         image, line_target = drawlinetwopoints(image, (cX,cY), (px, py), (255, 255, 0))


  #     if line_pole == None or line_gripper==None:
  #       rel_angle=np.NAN
  #       vel_angle=np.NAN
  #     else:
  #       tantheta=(line_pole[0]*line_gripper[1]-line_gripper[0]*line_pole[1])/(line_gripper[0]*line_pole[0]+line_gripper[1]*line_pole[1])
  #       rel_angle= np.arctan(tantheta)
  #       #print(np.rad2deg(rel_angle))
  #       rel_angle= np.rad2deg(rel_angle) - target_angle  
  #       rel_angle*=-1

  #       #calculate the angular velocity of the pole
        
  #       #print((rel_angle-self.pole_pos_old[0]))
  #       #print((rospy.get_rostime()-self.pole_pos_old[1]).to_sec())
  #       #TODO see if there is a better way to estimate the angle vel thats not so noisy ( window average?)
  #       vel_angle=(rel_angle-self.pole_pos_old[0])/(rospy.get_rostime()-self.pole_pos_old[1]).to_sec()

  #       #print(len(self.pole_vel_vec))
  #       self.pole_vel_vec.append(vel_angle)


  #       if len(self.pole_vel_vec)> self.pole_vel_window:
  #         pole_vel_vec_window=self.pole_vel_vec.pop(0)
  #       else:
  #         pole_vel_vec_window=self.pole_vel_vec

  #       vel_angle_windowed=np.mean(pole_vel_vec_window)

  #       if not self.pole_pos_old[1] ==rospy.get_rostime():
  #         self.pole_pos_old=(rel_angle,rospy.get_rostime())


  #     # #calculate acute agnel
  #     # rel_angle= np.arctan2(m_0-m_1,1+m_1*m_0)
  #     # rel_angle= np.rad2deg(rel_angle)

  #     #inpaing angle
  #     cv2.putText(image, str(np.round(rel_angle,2)) + " deg",
  #         (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
  #         0.5, (0, 255, 0), 2)

  #     cv2.putText(image, str(np.round(vel_angle_windowed,2)) + " deg/s",
  #         (50, 100), cv2.FONT_HERSHEY_SIMPLEX,
  #         0.5, (0, 255, 0), 2)

  #     #update observations
  #     if not np.isnan(rel_angle):
  #       self.msgPivotObservation.rel_angle = np.deg2rad(rel_angle)
  #     if not np.isnan(vel_angle_windowed):
  #       self.msgPivotObservation.pole_ang_vel = np.deg2rad(vel_angle_windowed)

  #   return image

#draws a long line through 2 points
def drawlinetwopoints(image, p1, p2, color):
  if not (p2[0]-p1[0])==0:
    A=float((p2[1]-p1[1]))
    B=float((p2[0]-p1[0]))
    m=A/B
    ptx=p1[0]-1000
    b=p1[1]-(m*p1[0])
    C=B*b
    pty=int(m*ptx +b)
    pbx=p2[0]+1000
    #b=bm[1]-(m*bm[0])
    pby=int(m*pbx +b)
  else:
    A=1
    B=0 
    C=-p1[0]
    ptx=p1[0]
    pty=0 
    pbx=p1[0]
    pby=image.shape[1]
  cv2.line(image, (ptx,pty), (pbx,pby), color, 2)

  return image , (A, B, C)


def drawline(image, tm,bm,mid):

  #extend the line in both directions
  if not (bm[0]-tm[0])==0:
    m=float((bm[1]-tm[1]))/float((bm[0]-tm[0]))
    ptx=tm[0]-1000
    b=tm[1]-(m*tm[0])
    pty=int(m*ptx +b)
    pbx=bm[0]+1000
    b=bm[1]-(m*bm[0])
    pby=int(m*pbx +b)
  else:
    m=np.Inf
    ptx=tm[0]
    pty=0 
    pbx=tm[0]
    pby=image.shape[1]

  if mid==0:
    cv2.line(image, (ptx,pty), (pbx,pby), (0, 0, 255), 2)
  if mid==1:
    cv2.line(image, (ptx,pty), (pbx,pby), (255, 0, 0), 2)

  return image, m






def main(args):
  rospy.init_node('pivot_angle', anonymous=True)

  pa = pivot_observer(target_angle=0)  

  r = rospy.Rate(1000) #1000hz
  while not rospy.is_shutdown():
      pa.pub_observations()
      r.sleep()
  # try:
  #   rospy.spin()
  # except KeyboardInterrupt:
  #   print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)