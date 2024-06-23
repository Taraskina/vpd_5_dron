#!/usr/bin/env python3

import time
from math import sin, cos, atan2

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Range # type: ignore

from hector_uav_msgs.srv import EnableMotors

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class SimpleMover():

    def __init__(self):
        rospy.init_node('simple_mover', anonymous=True)

        if rospy.has_param('gui'):
            self.enabled_gui = rospy.get_param('gui')
        else:
            rospy.logerr("Failed to get param 'gui'")

        self.cv_bridge = CvBridge()
        self.Image1 = None
        self.Image2 = None
        self.sonar_z = None
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("cam_1/camera/image", Image, self.camera_cb)
        rospy.Subscriber("cam_2/camera/image", Image, self.camera_cb2)
        rospy.Subscriber("/sonar_height", Range, self.sonar_cd)
        self.rate = rospy.Rate(30)


        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def sonar_cd(self, msg):
        self.sonar_z = msg.range
    

    def camera_cb(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.Image1 = cv_image
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def camera_cb2(self, msg):
        try:
            cv_image2 = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.Image2 = cv_image2
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def enable_motors(self):
        try:
            rospy.wait_for_service('enable_motors', 2)
            call_service = rospy.ServiceProxy('enable_motors', EnableMotors)
            response = call_service(True)
        except Exception as e:
            print("Error while try to enable motors: ")
            print(e)

    def points(self):
        image = cv2.resize(self.Image,(600,600))
        XY=[]
        for i in range(9,-1,-1):
            obrezimage = image[60*i:60*(i+1), 0:600]  
            BW=cv2.inRange(obrezimage,(0,0,0),(7,7,7))  
            contours=cv2.findContours(BW, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours=contours[0]
            if len(contours) > 1:
                contours=sorted(contours, key=cv2.contourArea, reverse=True)
                (x,y,w,h)=cv2.boundingRect(contours[0])
                (x2,y2,w2,h2)=cv2.boundingRect(contours[1])
                xx=(x+x2)//2+(w+w2)//4
                yy=60*i+(y+y2)//2+(h+h2)//4
                cv2.rectangle(image,(xx,yy), (xx+3, yy+3),(255,0,0), 5)
                #XY.append([(xx-300)//3,(yy-300)//(-3)])
                XY.append([xx, yy]) 
        return XY,image
    
    def pointsFront(image):
        BW=cv2.inRange(image,(0,0,0),(7,7,7))  
        contours=cv2.findContours(BW, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours=contours[0]
        if contours:
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contours[0])
        image = image[y:y+h, 0:820]
        XY = []
        dy=h//6
        for i in range (1,6):
            obrezimage = image[dy*i:dy*i+3, 0:820]
            BW=cv2.inRange(obrezimage,(0,0,0),(7,7,7))  
            contours=cv2.findContours(BW, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours=contours[0]
            if contours:
                contours=sorted(contours, key=cv2.contourArea, reverse=True)
                (x,y,w,h)=cv2.boundingRect(contours[0])
                (x2,y2,w2,h2)=cv2.boundingRect(contours[1])
                matrixX=sorted([x,x+w,x2,x2+w2])
                matrixY=sorted([y,y+h,y2,y2+h2])
                xx=(matrixX[1]+matrixX[2])//2
                yy=dy*i+(matrixY[1]+matrixY[2])//2
                cv2.rectangle(image,(xx,yy), (xx+3, yy+3),(255,0,0), 5)
                XY.append([(yy-308)//(-1),(xx-410)])
        return XY, image
    
    def RedCircle(self):
        image2 = cv2.resize(self.Image2,(600,600))
        BW=cv2.inRange(image2,(0,0,80),(10,10,255))  
        contours=cv2.findContours(BW, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours=contours[0]
        xy, pr =0, 0
        if contours:
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            cv2.drawContours(image2, contours, 0, (255,0,255), 5)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            pr=(w*h/(600*600))*100
            print(pr)
            xx=(x*2+w)//2
            yy=(y*2+h)//2
            cv2.rectangle(image2,(xx,yy), (xx+3, yy+3),(255,0,0), 5)
            cv2.imshow("555", BW)
            x=((x*2+w)//2-300)//3
            y=((y*2+h)//2-300)//(-3)
            xy=[x,y]
        return xy,pr,image2



    def take_off(self):
        self.enable_motors()
        start_time = time.time()
        end_time = start_time + 5
        twist_msg = Twist()
        twist_msg.linear.z = 1.0

        while (time.time() < end_time) and (not rospy.is_shutdown()):
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
    
    def main(self):
        self.enable_motors()
        goal_x, goal_y, goal_z = 0, 0, 1.7
        x, y, z = 0, 0, 0
        Vz, Vx, Vy = 0, 0, 0


        ##################################
        kp_z = 2; kd_z = 2
        kp_x = 0.5 * 10**(-2)
        kp_y = 6* 10**(-3)
        kp_angz = 1.5

        
        time_start = time.time()
        time_now = time_start
        e_now = goal_z

        flag_start_obleta = False
        time_conets_obleta = 0
        while not rospy.is_shutdown():
            twist_msg = Twist()

            #Z pose
            #rospy.loginfo(self.current_pose.position.z, self.sonar_height.range)
            
            #rospy.loginfo(f"Vz = {Vz}, sonar_z = {z}")
            if self.enabled_gui:
                
                x_goal, y_goal = 300, 300
                x_now, y_now = 300, 300
                y_goal_centr = 300
                x_goal_centr = 300
                centr_x, centr_y = 160, 120
                width, height = 0, 0
                xy, pr = 0, 0
                if self.Image1 is not None and self.Image2 is not None:
                    
                    
                    xy,pr, image2 = self.RedCircle()

                    XY, image1 = self.points()
                    try:
                        x_goal, y_goal = XY[-1]
                    except:
                        pass
                    
                    for el in XY:
                        #cv2.circle(image1, (el[0], el[1]), 5, (0, 0, 255), -1)
                        passImage
                    cv2.circle(image1, (x_goal, y_goal), 5, (0, 0, 255), -1)
                    cv2.circle(image1, (x_now, y_now), 5, (255, 0, 0), -1)
                    try:
                        cv2.circle(image1, (XY[5][0], XY[5][1]), 5, (255, 255, 0), -1)
                        cv2.circle(image1, (XY[4][0], XY[4][1]), 5, (255, 255, 0), -1)
                        x_goal_centr = (XY[4][0]+XY[5][0])//2
                        y_goal_centr = (XY[4][1] + XY[5][1])//2
                        cv2.circle(image1, (x_goal_centr, y_goal_centr), 5, (255, 255, 255), -1)
                    except:
                        pass


                    cv2.imshow("Down view camera from Robot", image1)
                    cv2.imshow("Front view camera from Robot", image2)
                    cv2.waitKey(3)

                    
            #Поворот
            #var = [x_goal, y_goal, x_now, y_now].copy()
            #y_goal, x_goal, y_now, x_now = var

            #вычисление ошибок
            ey = x_now - x_goal_centr#cenrta_x
            ex = y_now - y_goal
            etetta = atan2(x_now - x_goal, ex)
            
            #линейный xy, угловая z
            Vx, Vy = ex*kp_x, ey*kp_y
            Vz_ang = etetta * kp_angz


            #линейня z 
            if self.sonar_z is not None:
                z = self.sonar_z
            
            if pr > 20 and time.time() > time_conets_obleta:
                if not flag_start_obleta: flag_start_obleta = True
            if flag_start_obleta:
                time_conets_obleta = time.time() + 3
                flag_start_obleta = False
                
            
            if time.time() < time_conets_obleta:
                rospy.loginfo(f"\nЛЕТИМ ВЫСОКО!")
                goal_z = 3
            else:
                goal_z = 1.7
            


            e_last = e_now 
            e_now = goal_z - z
            time_last = time_now
            time_now = time.time()
            Vz = e_now * kp_z + (e_now - e_last) * (time_now - time_last) * kd_z
            twist_msg.linear.z = Vz


            rospy.loginfo(f"e = {ex, ey}\nV= {Vx, Vy, Vz},\n w = {Vz_ang}, z={z}\n {pr}")
            if time.time() - time_start >= 5:
                twist_msg.linear.x = Vx
                pass
            twist_msg.angular.z = Vz_ang
            twist_msg.linear.y = Vy
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()

        def spin(self):
            self.take_off()
            start_time = time.time()
            acceleration = 0.2  # Ускорение в м/с²
            max_speed = 1.0  # Максимальная скорость в м/с

            while not rospy.is_shutdown():
                twist_msg = Twist()
                t = time.time() - start_time
                speed = min(acceleration * t, max_speed)
                twist_msg.linear.z = speed

                # Обработка данных с передней камеры
                if self.Image2 is not None:
                    XY = self.Image2  # Массив точек из передней камеры
                    
                    if self.Image2 is not None:
                        XY = self.Image2  # Массив точек из передней камеры

                    # Находим центр масс точек
                    center_x = np.mean(points[:, 0])
                    center_y = np.mean(points[:, 1])

                    # Определяем направление движения на основе центра масс
                    if center_x > image_width / 2:
                        # Центр масс справа, поворачиваем влево
                        twist_msg.angular.z = -0.1
                    elif center_x < image_width / 2:
                        # Центр масс слева, поворачиваем вправо
                        twist_msg.angular.z = 0.1
                    else:
                        # Центр масс по центру, не поворачиваем
                        twist_msg.angular.z = 0.0
                    
                    # Управление поворотом (пример, добавить поворот вправо)
                    twist_msg.angular.z = 0.1  # Скорость поворота в рад/с
                
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()
                
                if self.enabled_gui:
                    if self.Image1 is not None and self.Image2 is not None:
                        cv2.imshow("Down view camera from Robot", self.Image1)
                        cv2.imshow("Front view camera from Robot", self.Image2)
                        cv2.waitKey(3)

            

            def check_moving(self):
                self.take_off()
                rospy.loginfo(f"Поднялись в верх") 
                time_now= time.time()
                time_start = time.time()
                goal_x, goal_y, goal_z =  0, 0, 0
                p = 0.01
                x, y, z, phi, th, fi = [0, 0, 0, 0, 0, 0]
                Vx, Vy, Vz = 0, 0, 0
                rospy.loginfo(f"Поднялись в верх") 
                dtime = 0.1
                while not rospy.is_shutdown():
                    twist_msg = Twist()
                    '''t = time.time() - start_time
                    twist_msg.linear.z = 0.8 * cos(1.2 * t)
                    twist_msg.linear.y = 0.8 * sin(0.6 * t)
                    self.cmd_vel_pub.publish(twist_msg)
                    self.rate.sleep()'''

                    #Рабочий код
                    Vx_last = Vx
                    Vy_last = Vy
                    Vz_last = Vz
                
                    Vx = p * (goal_x - x)
                    Vy = p * (goal_y - y)
                    Vz = p * (goal_z - z)
                
                    twist_msg.linear.z = Vz
                    twist_msg.linear.x = 0
                    twist_msg.linear.y = 0
                    rospy.loginfo(Vx, Vy, Vz) 
                    time_last = time_now
                    time_now = time.time()
                    dtime = (time_now - time_last)
                    z += (Vz_last + Vz)/2 * dtime
                    x += (Vx_last + Vx)/2 * dtime
                    y += (Vy_last + Vy)/2 * dtime
                    rospy.loginfo(f"Vz = {Vz}; z = {z}")
                    self.cmd_vel_pub.publish(twist_msg)
                    self.rate.sleep()

                    if self.enabled_gui:
                        if self.Image1 is not None and self.Image2 is not None:
                            cv2.imshow("Down view camera from Robot", self.Image1)
                            cv2.imshow("Front view camera from Robot", self.Image2)
                            cv2.waitKey(3)

    


   

simple_mover = SimpleMover()
simple_mover.main()
