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
        #self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback_follow_open)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()
        self.prev_direction = "left"
        self.still_turning = False

    def laser_callback_follow_open(self, msg):
        ranges = [x for x in msg.ranges if str(x) != 'nan']
        far_left_dist = self.get_range_far_left_dist(ranges)
        left_dist = self.get_range_left_dist(ranges)
        middle_dist = self.get_range_middle_dist(ranges)
        right_dist = self.get_range_right_dist(ranges)
        far_right_dist = self.get_range_far_right_dist(ranges)
        min_dist = min(ranges)
        min_middle_dist = self.get_min_middle_dist(ranges)
        distances = [far_left_dist, left_dist, middle_dist, right_dist, far_right_dist]
        print "min dist", min_middle_dist
        if min_middle_dist > 1:
            max_vel = 0.5
        else:
            max_vel = 0.25

        if self.still_turning == True:
            print "still turning"
            print min_dist
            print self.prev_direction
            if min_dist > 0.4:
                self.twist.angular.z = 0
                self.still_turning = False
                time.sleep(1)
            elif self.prev_direction == 'right':
                self.twist.angular.z = 1
            elif self.prev_direction == 'left':
                self.twist.angular.z = -1
        else:
            if min_dist > 0.4:
                print "furthest side", max(range(len(distances)), key=distances.__getitem__)
                if 0 == max(range(len(distances)), key=distances.__getitem__) or 4 == min(range(len(distances)), key=distances.__getitem__):
                    print "moving hard left"
                    self.twist.linear.x = max_vel/2
                    self.twist.angular.z = -0.75

                elif 1 == max(range(len(distances)), key=distances.__getitem__):
                    print "moving left"
                    self.twist.linear.x = max_vel
                    self.twist.angular.z = -0.5

                elif 2 == max(range(len(distances)), key=distances.__getitem__):
                    print "moving forward"
                    self.twist.linear.x = max_vel
                    self.twist.angular.z = 0

                elif 3 == max(range(len(distances)), key=distances.__getitem__):
                    print "moving right"
                    self.twist.linear.x = max_vel
                    self.twist.angular.z = 0.5

                elif 4 == max(range(len(distances)), key=distances.__getitem__) or 0 == min(range(len(distances)), key=distances.__getitem__):
                    print "moving hard right"
                    self.twist.linear.x = max_vel/2
                    self.twist.angular.z = 0.75

            else:
                self.still_turning = True
                self.twist.linear.x = 0
                if (far_left_dist + left_dist)/2 > (far_right_dist + right_dist)/2:
                    print "turning left"
                    self.twist.angular.z = -0.5
                    self.prev_direction = 'left'
                else:
                    print "turning right"
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

    def get_range_far_left_dist(self, ranges):
        left = ranges[0:(len(ranges)//5)]
        return sum(left)/len(left)

    def get_range_left_dist(self, ranges):
        left = ranges[(len(ranges)//5):((len(ranges)//5)*2)]
        return sum(left)/len(left)

    def get_range_right_dist(self, ranges):
        right = ranges[((len(ranges) // 5)*3):((len(ranges) // 5)*4)]
        return sum(right)/len(right)

    def get_range_far_right_dist(self, ranges):
        right = ranges[((len(ranges) // 5)*4):(len(ranges))]
        return sum(right)/len(right)

    def get_range_middle_dist(self, ranges):
        middle = ranges[((len(ranges) // 5)*2):((len(ranges) // 5)*3)]
        return sum(middle)/len(middle)

    def get_min_middle_dist(self, ranges):
        middle = ranges[(len(ranges) // 5):((len(ranges) // 5)*4)]
        return min(middle)

    def image_callback(self, data):
        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # create HSV colour space
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        blue_hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((134, 60, 60)),
                                 numpy.array((184, 255, 255)))

        red_hsv_thresh = cv2.inRange(hsv_img,
                                numpy.array((227, 60, 60)),
                                numpy.array((21, 255, 255)))

        green_hsv_thresh = cv2.inRange(hsv_img,
                               numpy.array((53, 60, 60)),
                               numpy.array((110, 255, 255)))

        # find the contours in the mask generated from the HSV image.
        _, blue_hsv_contours, hierachy = cv2.findContours(
            blue_hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        _, red_hsv_contours, hierachy = cv2.findContours(
            red_hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        _, green_hsv_contours, hierachy = cv2.findContours(
            green_hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        # iterate over all those found contours and calculate area
        for c in blue_hsv_contours:
            a = cv2.contourArea(c)
            # if the area is big enough, we draw the outline
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)
                print "i see blue"

        for c in red_hsv_contours:
            a = cv2.contourArea(c)
            # if the area is big enough, we draw the outline
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (0, 255, 0), 3)
                print "i see red"

        for c in green_hsv_contours:
            a = cv2.contourArea(c)
            # if the area is big enough, we draw the outline
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (0, 0, 255), 3)
                print "i see green"

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


#cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
