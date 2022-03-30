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
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback_follow_open)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()
        self.prev_direction = "left"
        self.still_turning = False

    def laser_callback_follow_open(self, msg):
        ranges = [x for x in msg.ranges if str(x) != 'nan']
        left_dist = self.get_range_left_dist(ranges)
        middle_dist = self.get_range_middle_dist(ranges)
        right_dist = self.get_range_right_dist(ranges)
        min_dist = min(ranges)

        if self.still_turning == True:
            print "still turning"
            print dist
            print self.prev_direction
            if min_dist > 0.5:
                self.twist.angular.z = 0
                self.still_turning = False
                time.sleep(1)
            elif self.prev_direction == 'right':
                self.twist.angular.z = 0.5
            elif self.prev_direction == 'left':
                self.twist.angular.z = -0.5
        else:
            if min_dist > 0.5:
                if middle_dist > left_dist and middle_dist > right_dist:
                    print "moving forward"
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = 0
                elif left_dist > middle_dist and left_dist > right_dist:
                    print "moving left"
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = -0.5
                elif right_dist > middle_dist and right_dist > left_dist:
                    print "moving right"
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = 0.5

            else:
                self.still_turning = True
                print "turning"
                self.twist.linear.x = 0
                if left_dist > right_dist:
                    print "moving left"
                    self.twist.angular.z = -0.5
                    self.prev_direction = 'left'
                else:
                    print "moving right"
                    self.twist.angular.z = 0.5
                    self.prev_direction = 'right'

        self.cmd_vel_pub.publish(self.twist)

    def laser_callback_left_right(self, msg):
        ranges = [x for x in msg.ranges if str(x) != 'nan']
        dist = self.get_range_middle_dist(ranges)
        if self.still_turning == True:
            print "still turning"
            print dist
            print self.prev_direction
            if dist > 0.65:
                self.twist.angular.z = 0
                self.still_turning = False
                time.sleep(1)
            elif self.prev_direction == 'right':
                self.twist.angular.z = 1.5
            elif self.prev_direction == 'left':
                self.twist.angular.z = -1.5

        else:
            if dist > 0.65:
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

    def get_range_left_dist(self, ranges):
        # computing strt, and end index
        strt_idx = 0
        end_idx = len(ranges)//3

        # using loop to get indices
        left = []
        for idx in range(len(ranges)):

            # checking for elements in range
            if idx >= strt_idx and idx <= end_idx:
                left.append(ranges[idx])

        return sum(left)/len(left)

    def get_range_right_dist(self, ranges):
        # computing strt, and end index
        strt_idx = (len(ranges) // 3)*2
        end_idx = len(ranges)

        # using loop to get indices
        right = []
        for idx in range(len(ranges)):

            # checking for elements in range
            if idx >= strt_idx and idx <= end_idx:
                right.append(ranges[idx])

        return sum(right)/len(right)

    def get_range_middle_dist(self, ranges):
        # computing strt, and end index
        strt_idx = (len(ranges) // 3)
        end_idx = (len(ranges) // 3)*2

        # using loop to get indices
        middle = []
        for idx in range(len(ranges)):

            # checking for elements in range
            if idx >= strt_idx and idx <= end_idx:
                middle.append(ranges[idx])

        return sum(middle)/len(middle)


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
