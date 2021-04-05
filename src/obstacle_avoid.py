#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from math import pi

class Follower:

    global base_speed
    global max_Lanes
    global current_Lane

    def __init__(self):
        self.base_speed = 1 #m/s
        self.max_Lanes = 2
        self.current_Lane = 1
        self.changing_lane = False
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        #self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.state = "STARTING"
        self.logcount = 0

    def image_callback(self, msg):

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)

        # filter out everything that's not Asphalt black
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_gray = numpy.array([ 5, 5, 50])
        upper_gray = numpy.array([ 40, 40, 80])

        mask = cv2.inRange(hsv,  lower_gray, upper_gray)
        masked = cv2.bitwise_and(image, image, mask=mask)

        # clear all but a 20 pixel band near the top of the image
        
        h, w, d = image.shape

        search_top = h * 1/4
        search_bot = search_top + 400
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        resized_mask = cv2.resize(mask, (480,270))
        cv2.imshow("band", resized_mask)
        print ("pixel value: " + str(hsv[7*h/8, w/2]))
        # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))
        if M['m00'] > 0:
            self.state = "FOLLOWING_LINE"
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # add a turn if the centroid is not in the center
            err = cx - w/2
            self.twist.linear.x = self.base_speed
            self.twist.angular.z = -float(err) / 400
            self.cmd_vel_pub.publish(self.twist)
        else: #patrolling
            self.patrol()
        print("state: ", self.state)
        resized = cv2.resize(image, (480,270))
        cv2.imshow("image", resized)
        cv2.waitKey(3)

    # This method simply let the robot to patrol in straight line
    def patrol(self):
        self.state = "PATROLLING"
        self.twist.linear.x = self.base_speed
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

rospy.init_node('follower')
follower = Follower()
rospy.spin()