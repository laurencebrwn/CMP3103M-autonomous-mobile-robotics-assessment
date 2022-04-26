#!/usr/bin/env python
# run: roslaunch uol_turtlebot_simulator maze1.launch
# then run this py file

# import nessacary libraries
import numpy
import cv2
import cv_bridge
import rospy
import time
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# define the class that the robot uses to navigate
class Follower:
    def __init__(self):
        # instantiate the CV2 bridge, subscribers and publishers
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback_follow_open)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        # instantiate various variables for the robot to keep track of its movements
        self.twist = Twist()
        self.prev_direction = "left" # keep track of the direction that the robot is turning in
        self.still_turning = False # keep track of if the robot should still be turning
        self.moving_from_red = [False,""] # if the robot has seen red, flag it and store the direction red is from the robot
        self.moving_to_green = [False,""] # if the robot has seen green, flag it and store the direction green is from the robot
        self.moving_to_blue = [False,""] # if the robot has seen blue, flag it and store the direction blue is from the robot
        self.final_route_stated = False # keep track if the robot is on its final stretch
        self.finished = False # keep track if the robot has reached its target
        self.green_close = False # keep track if the robot is close to green

    # the laser callback that drives the robots movement based on distance readings and information saved from the image callback
    def laser_callback_follow_open(self, msg):
        # extract the distances taken from the laser subscriber
        ranges = [x for x in msg.ranges if str(x) != 'nan']

        # if not reached green (finished) enter the main movement chain
        if self.finished == False:
            # if the robot is moving from red enter red_movement mode
            if self.moving_from_red[0] == True:
                self.red_movement(ranges)
            # else, if the robot has reached green, stop the robot and declare finished state
            elif self.green_close == True and self.moving_to_green[0] == False:
                self.finished = True
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                print "finished" # debug
            # else, if the robot has seen green enter the green_movement mode
            elif self.moving_to_green[0] == True:
                self.green_movement(ranges)
            # else, if the robot has seen blue enter the blue_movement mode
            elif self.moving_to_blue[0] == True and self.final_route_stated == False:
                self.blue_movement(ranges)
            # else, if the robot has not seen green, blue or red enter the normal_movement mode
            elif self.green_close == False:
                self.normal_movement(ranges)

            # publish the movement commands set by the above modes to the robot
            self.cmd_vel_pub.publish(self.twist)

    # the red_movement mode, the robot enters when it has red in its vision
    def red_movement(self, ranges):
        # dont move anymore forward
        self.twist.linear.x = 0
        # flag the robot as still turning, so that the robot completes the turn
        self.still_turning = True
        # based on if the robot has seen red in either the left or the right side of its view, turn the respective direction
        if self.moving_from_red[1] == "right":
            self.twist.angular.z = 1
            self.prev_direction = 'left'
        else:
            self.twist.angular.z = -1
            self.prev_direction = 'right'

    # the green_movement mode, the robot enters when it has red in its vision
    def green_movement(self, ranges):
        # get the distances from the laser scanner, split into 5 sections
        far_left_dist = self.get_range_far_left_dist(ranges)
        left_dist = self.get_range_left_dist(ranges)
        right_dist = self.get_range_right_dist(ranges)
        far_right_dist = self.get_range_far_right_dist(ranges)
        # get the minimum distance that an object is from the robot, for crash detection
        min_dist = min(ranges)
        # flag that the robot is on its final stretch
        self.final_route_stated = True

        # if the robot is not too close to a wall, move towards the green
        if min_dist > 0.4:
            # if the robot sees green straight ahead, move normally (forward)
            if self.moving_to_green[1] == "both" or self.still_turning == True:
                self.normal_movement(ranges)
            # if the robot sees green to the right, move right towards it
            elif self.moving_to_green[1] == "right":
                self.twist.linear.x = 0.25
                self.twist.angular.z = -1
                self.prev_direction = 'right'
            # if the robot sees green to the left, move left towards it
            else:
                self.twist.linear.x = 0.25
                self.twist.angular.z = 1
                self.prev_direction = 'left'

        # if the robot is too close to a wall, avoid it
        else:
            # flag the robot as still turning, so that the robot completes the turn
            self.still_turning = True
            # dont move anymore forward
            self.twist.linear.x = 0
            # if the robot is REALLY close, back away
            if min_dist < 0.32:
                self.twist.linear.x = -0.25
            # else, if the wall is closer to the right hand side of the robot, turn left
            elif (far_left_dist + left_dist)/2 > (far_right_dist + right_dist)/2:
                print "turning left" # debug
                self.twist.angular.z = 0.5
                self.prev_direction = 'left'
            # else, if the wall is closer to the left hand side of the robot, turn right
            else:
                print "turning right" # debug
                self.twist.angular.z = -0.5
                self.prev_direction = 'right'

    # the blue_movement mode, the robot enters when it has blue in its vision
    def blue_movement(self, ranges):
        # get the minimum distance that an object is from the robot, for crash detection / if it has reached the blue square
        min_dist = min(ranges)
        print ranges # debug
        # if walls are close, or blue is straight ahead, move normally
        if min_dist < 1 or self.moving_to_blue[1] == "both":
            self.normal_movement(ranges)
        # else, if blue is to the right, turn right
        elif self.moving_to_blue[1] == "right":
            self.twist.linear.x = 0.25
            self.twist.angular.z = -0.5
            self.prev_direction = 'right'
        # else, if blue is to the left, turn left
        else:
            self.twist.linear.x = 0.25
            self.twist.angular.z = 0.5
            self.prev_direction = 'left'

    # the normal_movement mode, the robot enters when it doesn't have any colours of interest in its vision
    def normal_movement(self, ranges):
        # get the distances from the laser scanner, split into 5 sections
        far_left_dist = self.get_range_far_left_dist(ranges)
        left_dist = self.get_range_left_dist(ranges)
        middle_dist = self.get_range_middle_dist(ranges)
        right_dist = self.get_range_right_dist(ranges)
        far_right_dist = self.get_range_far_right_dist(ranges)
        # get the minimum distance that an object is from the robot, for crash detection
        min_dist = min(ranges)
        # get the minimum distance that an object is IN FRONT of robot, for speed control
        min_middle_dist = self.get_min_middle_dist(ranges)
        # store distances in a list
        distances = [far_left_dist, left_dist, middle_dist, right_dist, far_right_dist]
        print "min dist", min_middle_dist # debug

        # speed control, allowing for faster speeds if the robot is not in imminent danger
        if min_middle_dist > 1:
            max_vel = 0.5
        else:
            max_vel = 0.25

        # if the robot has been marked as still turning, keep turning until safe to continue
        if self.still_turning == True:
            print "still turning" # debug
            print self.prev_direction # debug
            # if the closest distance from an object is great enough, stop turning
            if min_dist > 0.8:
                self.twist.angular.z = 0
                self.still_turning = False
            # else, if the closest distance from an object is too close, back up
            elif min_dist < 0.32:
                self.twist.linear.x = -0.25
            # else, if the robot is turning right, continue to do so
            elif self.prev_direction == 'right':
                self.twist.linear.x = 0
                self.twist.angular.z = -1
            # else, if the robot is turning left, continue to do so
            elif self.prev_direction == 'left':
                self.twist.linear.x = 0
                self.twist.angular.z = 1

        # if the robot is not marked as still turning, figure out how to best move next
        else:
            # if there are no objects too close to the robot, move
            if min_dist > 0.4:
                #  if the robot is stuck in a corner, if so turn around the direction of most openness
                if middle_dist < 0.8 and middle_dist > left_dist and middle_dist > right_dist:
                    print "stuck in corner" # debug
                    self.still_turning = True
                    self.twist.linear.x = 0
                    if self.prev_direction == 'right':
                        print "turning left" # debug
                        self.twist.angular.z = 0.5
                        self.prev_direction = 'left'
                    else:
                        print "turning right" # debug
                        self.twist.angular.z = -0.5
                        self.prev_direction = 'right'

                # else, if the far left laser scan area has the furthest distance until a wall, move that direction
                elif 0 == max(range(len(distances)), key=distances.__getitem__) or 4 == min(range(len(distances)), key=distances.__getitem__):
                    # robot turns hard left, if the route ahead is not perfectly adequete
                    if far_left_dist > (middle_dist*1.5) or far_left_dist < 8:
                        print "moving hard left" # debug
                        self.twist.linear.x = max_vel/2
                        self.twist.angular.z = 0.75
                        self.prev_direction = 'left'
                    # else, if the route ahead is fine, move forward
                    else:
                        print "moving forward" # debug
                        self.twist.linear.x = max_vel
                        self.twist.angular.z = 0

                # else, if the left laser scan area has the furthest distance until a wall, move that direction
                elif 1 == max(range(len(distances)), key=distances.__getitem__):
                    # robot turns left, if the route ahead is not perfectly adequete
                    if left_dist > (middle_dist*1.5) or left_dist < 8:
                        print "moving left" # debug
                        self.twist.linear.x = max_vel
                        self.twist.angular.z = 0.5
                        self.prev_direction = 'left'
                    # else, if the route ahead is fine, move forward
                    else:
                        print "moving forward" # debug
                        self.twist.linear.x = max_vel
                        self.twist.angular.z = 0

                # else, if the center laser scan area has the furthest distance until a wall, move that forward
                elif 2 == max(range(len(distances)), key=distances.__getitem__):
                    print "moving forward" # debug
                    self.twist.linear.x = max_vel
                    self.twist.angular.z = 0

                # else, if the right laser scan area has the furthest distance until a wall, move that direction
                elif 3 == max(range(len(distances)), key=distances.__getitem__):
                    # robot turns right, if the route ahead is not perfectly adequete
                    if right_dist > (middle_dist*1.5) or right_dist < 8:
                        print "moving right" # debug
                        self.twist.linear.x = max_vel
                        self.twist.angular.z = -0.5
                        self.prev_direction = 'right'
                    # else, if the route ahead is fine, move forward
                    else:
                        print "moving forward" # debug
                        self.twist.linear.x = max_vel
                        self.twist.angular.z = 0

                # else, if the right laser scan area has the furthest distance until a wall, move that direction
                elif 4 == max(range(len(distances)), key=distances.__getitem__) or 0 == min(range(len(distances)), key=distances.__getitem__):
                    # robot turns hard right, if the route ahead is not perfectly adequete
                    if far_right_dist > (middle_dist*1.5) or far_right_dist < 8:
                        print "moving hard right" # debug
                        self.twist.linear.x = max_vel/2
                        self.twist.angular.z = -0.75
                        self.prev_direction = 'right'
                    # else, if the route ahead is fine, move forward
                    else:
                        print "moving forward" # debug
                        self.twist.linear.x = max_vel
                        self.twist.angular.z = 0

            # if there are objects too close to the robot, move away
            else:
                # flag the robot as still turning, so that the robot completes the turn
                self.still_turning = True
                # if the robot is REALLY close, back away
                if min_dist < 0.32:
                    self.twist.linear.x = -0.25
                # else, if the wall is closer to the right hand side of the robot, turn left
                elif (far_left_dist + left_dist)/2 > (far_right_dist + right_dist)/2:
                    print "turning left" # debug
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0.5
                    self.prev_direction = 'left'
                # else, if the wall is closer to the left hand side of the robot, turn right
                else:
                    print "turning right" # debug
                    self.twist.linear.x = 0
                    self.twist.angular.z = -0.5
                    self.prev_direction = 'right'

    # get the average distance of walls on the far left side of the robot
    def get_range_far_left_dist(self, ranges):
        left = ranges[((len(ranges) // 5)*4):(len(ranges))]
        return sum(left)/len(left)

    # get the average distance of walls on the left side of the robot
    def get_range_left_dist(self, ranges):
        left = ranges[((len(ranges) // 5)*3):((len(ranges) // 5)*4)]
        return sum(left)/len(left)

    # get the average distance of walls on the right side of the robot
    def get_range_right_dist(self, ranges):
        right = ranges[(len(ranges)//5):((len(ranges)//5)*2)]
        return sum(right)/len(right)

    # get the average distance of walls on the far right side of the robot
    def get_range_far_right_dist(self, ranges):
        right = ranges[0:(len(ranges)//5)]
        return sum(right)/len(right)

    # get the average distance of walls in front of the robot
    def get_range_middle_dist(self, ranges):
        middle = ranges[((len(ranges) // 5)*2):((len(ranges) // 5)*3)]
        return sum(middle)/len(middle)

    # get the minimum distance of walls in front of the robot
    def get_min_middle_dist(self, ranges):
        middle = ranges[(len(ranges) // 5):((len(ranges) // 5)*4)]
        return min(middle)

    # the image callback that detects red, green and blue object, from the robots camera, to inform its movement
    def image_callback(self, data):
        if self.finished == False:
            # try to instantiate the CV2 bridge, using the BGR colour space
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError, e:
                print e

            # instantiate variables that mark whether, and if so, where colours are seen
            seen_red = False
            red_dir = ""
            seen_blue = False
            blue_dir = ""
            seen_green = False
            green_dir = ""
            green_left_a = 0
            blue_left_a = 0

            # crop image for red floor tiles
            dimensions = cv_image.shape
            height = cv_image.shape[0]
            width = cv_image.shape[1]
            cropped_cv_image_red_l = cv_image[((height//20)*17):((height//20)*18), 0:(width//2)]
            cropped_cv_image_red_r = cv_image[((height//20)*17):((height//20)*18), (width//2):width]

            # crop image into left and right for green and blue get_min_middle_dist
            cropped_cv_image_l = cv_image[0:height, 0:(width//2)]
            cropped_cv_image_r = cv_image[0:height, (width//2):width]

            # crop image for close to green
            cropped_cv_image_green = cv_image[((height//20)*17):((height//20)*18), (width//3):((width//3)*2)]

            # create HSV colour space
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            cropped_hsv_img_red_l = cv2.cvtColor(cropped_cv_image_red_l, cv2.COLOR_BGR2HSV)
            cropped_hsv_img_red_r = cv2.cvtColor(cropped_cv_image_red_r, cv2.COLOR_BGR2HSV)
            cropped_hsv_img_l = cv2.cvtColor(cropped_cv_image_l, cv2.COLOR_BGR2HSV)
            cropped_hsv_img_r = cv2.cvtColor(cropped_cv_image_r, cv2.COLOR_BGR2HSV)
            cropped_hsv_img_green = cv2.cvtColor(cropped_cv_image_green, cv2.COLOR_BGR2HSV)

            # calculate colour thresholds
            blue_hsv_thresh_left = cv2.inRange(cropped_hsv_img_l,
                                     numpy.array((110, 50, 50)),
                                     numpy.array((130, 255, 255)))
            blue_hsv_thresh_right = cv2.inRange(cropped_hsv_img_r,
                                     numpy.array((110, 50, 50)),
                                     numpy.array((130, 255, 255)))

            red_hsv_thresh_l1 = cv2.inRange(cropped_hsv_img_red_l,
                                    numpy.array((160, 50, 50)),
                                    numpy.array((180, 255, 255)))
            red_hsv_thresh_l2 = cv2.inRange(cropped_hsv_img_red_l,
                                    numpy.array((0, 50, 50)),
                                    numpy.array((20, 255, 255)))
            red_hsv_thresh_r1 = cv2.inRange(cropped_hsv_img_red_r,
                                    numpy.array((160, 50, 50)),
                                    numpy.array((180, 255, 255)))
            red_hsv_thresh_r2 = cv2.inRange(cropped_hsv_img_red_r,
                                    numpy.array((0, 50, 50)),
                                    numpy.array((20, 255, 255)))
            red_hsv_thresh_left = red_hsv_thresh_l1 +red_hsv_thresh_l2
            red_hsv_thresh_right = red_hsv_thresh_r1 +red_hsv_thresh_r2

            green_hsv_thresh_left = cv2.inRange(cropped_hsv_img_l,
                                   numpy.array((50, 50, 50)),
                                   numpy.array((70, 255, 255)))
            green_hsv_thresh_right = cv2.inRange(cropped_hsv_img_r,
                                   numpy.array((50, 50, 50)),
                                   numpy.array((70, 255, 255)))
            green_hsv_thresh_close = cv2.inRange(cropped_hsv_img_green,
                                   numpy.array((50, 50, 50)),
                                   numpy.array((70, 255, 255)))

            # find the contours in the mask generated from the HSV image.
            _, blue_hsv_contours_left, hierachy = cv2.findContours(
                blue_hsv_thresh_left.copy(),
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)
            _, blue_hsv_contours_right, hierachy = cv2.findContours(
                blue_hsv_thresh_right.copy(),
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)

            _, red_hsv_contours_left, hierachy = cv2.findContours(
                red_hsv_thresh_left.copy(),
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)
            _, red_hsv_contours_right, hierachy = cv2.findContours(
                red_hsv_thresh_right.copy(),
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)

            _, green_hsv_contours_left, hierachy = cv2.findContours(
                green_hsv_thresh_left.copy(),
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)
            _, green_hsv_contours_right, hierachy = cv2.findContours(
                green_hsv_thresh_right.copy(),
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)
            _, green_hsv_contours_close, hierachy = cv2.findContours(
                green_hsv_thresh_close.copy(),
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)


            # iterate over all those found contours and calculate area
            for c in blue_hsv_contours_left:
                a = cv2.contourArea(c)
                # if the area is big enough flag it has seen the colour, and its direction
                if a > 100.0:
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)
                    print "i see blue:", a,"%" # debug
                    seen_blue = True
                    blue_dir = "left"
                    blue_left_a = a

            for c in blue_hsv_contours_right:
                a = cv2.contourArea(c)
                # if the area is big enough flag it has seen the colour, and its direction
                if a > 100.0:
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)
                    print "i see blue:", a,"%" # debug
                    seen_blue = True
                    # if the colour is seen in both left and right sides, it must be ahead, so flag it so
                    if blue_left_a > 1000 and  a > 1000:
                        blue_dir = "forward"
                    else:
                        blue_dir = "right"

            for c in red_hsv_contours_left:
                a = cv2.contourArea(c)
                # if the area is big enough flag it has seen the colour, and its direction
                if a > 1000.0:
                    cv2.drawContours(cv_image, c, -1, (0, 0, 255), 3)
                    print "i see red:", a,"%" # debug
                    seen_red = True
                    red_dir = "left"

            for c in red_hsv_contours_right:
                a = cv2.contourArea(c)
                # if the area is big enough flag it has seen the colour, and its direction
                if a > 1000.0:
                    cv2.drawContours(cv_image, c, -1, (0, 0, 255), 3)
                    print "i see red:", a,"%" # debug
                    seen_red = True
                    red_dir = "right"

            for c in green_hsv_contours_left:
                a = cv2.contourArea(c)
                # if the area is big enough flag it has seen the colour, and its direction
                if a > 1000.0:
                    cv2.drawContours(cv_image, c, -1, (0, 255, 0), 3)
                    print "i see green:", a,"%" # debug
                    seen_green = True
                    green_dir = "left"
                    green_left_a = a

            for c in green_hsv_contours_right:
                a = cv2.contourArea(c)
                # if the area is big enough flag it has seen the colour, and its direction
                if a > 1000.0:
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)
                    print "i see green:", a,"%" # debug
                    seen_green = True
                    # if the colour is seen in both left and right sides, it must be ahead, so flag it so
                    if green_left_a > 4000 and a > 4000:
                        green_dir = "forward"
                    else:
                        green_dir = "right"

            for c in green_hsv_contours_close:
                a = cv2.contourArea(c)
                # if the area is big enough flag it has seen the colour, and its direction
                if a > 100.0:
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)
                    print "green is very close!" # debug
                    self.green_close = True

            # feed back all the findings from the camera to the classes variables, for the laser callback to access
            self.moving_from_red = [seen_red, red_dir]
            self.moving_to_green = [seen_green, green_dir]
            self.moving_to_blue = [seen_blue, blue_dir]

            cv2.waitKey(1)


#cv2.startWindowThread()  # debug
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
