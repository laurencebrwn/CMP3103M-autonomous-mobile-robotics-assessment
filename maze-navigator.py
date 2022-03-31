#!/usr/bin/env python
# run: roslaunch uol_turtlebot_simulator maze1.launch
# then run this py file

import numpy

import cv2
import cv_bridge
import rospy
import time
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
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
            print min_dist
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
        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # create HSV colour space
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((0, 150, 50)),
                                 numpy.array((255, 255, 255)))

        # find the contours in the mask generated from the HSV image.
        _, hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        # in hsv_contours we now have an array of individual
        # closed contours (basically a polgon around the
        # blobs in the mask). Let's iterate over all those found
        # contours.
        for c in hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            a = cv2.contourArea(c)
            # and if the area is big enough, we draw the outline
            # of the contour (in blue)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


#cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
