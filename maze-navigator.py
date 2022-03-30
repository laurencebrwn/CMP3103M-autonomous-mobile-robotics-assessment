#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy
import time

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()
        self.prev_direction = "left"
        self.still_turning = False

    def laser_callback(self, msg):
        ranges = [x for x in msg.ranges if str(x) != 'nan']
        dist = min(ranges)
        if self.still_turning == True:
            print "still turning"
            print dist
            print self.prev_direction
            if dist > 1:
                self.twist.angular.z = 0
                self.still_turning = False
                time.sleep(1)
            elif self.prev_direction == 'right':
                self.twist.angular.z = 1.5
            elif self.prev_direction == 'left':
                self.twist.angular.z = -1.5

        else:
            if dist > 1:
                print "moving"
                self.twist.linear.x = 0.5
                self.twist.angular.z = 0

            else:
                self.still_turning = True
                print "turning"
                self.twist.linear.x = 0
                if self.prev_direction == 'left':
                    self.twist.angular.z = 1.5
                    self.prev_direction = 'right'
                else:
                    self.twist.angular.z = -1.5
                    self.prev_direction = 'left'

        self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            print self.twist.angular.z

            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)


#cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
