#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

    def image_callback(self, msg):

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        '''
        lower_red1 = numpy.array([ 110, 0, 100])
        upper_red1 = numpy.array([ 255, 255, 255])
        lower_red2 = numpy.array([ 0, 0, 100])
        upper_red2 = numpy.array([ 50, 255, 255])
        '''
        lower_gray = numpy.array([ 5, 5, 50])
        upper_gray = numpy.array([ 40, 40, 80])
        #red_mask1 = cv2.inRange(hsv,  lower_red1, upper_red1)
        #red_mask2 = cv2.inRange(hsv,  lower_red2, upper_red2)
        #mask = cv2.bitwise_or(red_mask1, red_mask2)

        mask = cv2.inRange(hsv,  lower_gray, upper_gray)
        masked = cv2.bitwise_and(image, image, mask=mask)

        #masked = cv2.bitwise_and(image, image, mask=mask)

    # clear all but a 20 pixel band near the top of the image
        
        h, w, d = image.shape
        '''
        search_top = 1 * h /2 + 30
        search_bot = search_top + 100
        search_left = 5 * w / 8
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[search_top:search_bot, search_left:w] = 0
        '''
        search_top = h / 2
        search_bot = h - 400
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        cv2.imshow("band", mask)
        print "pixel value: " + str(hsv[7*h/8, w/2])
    # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) - 300
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # add a turn if the centroid is not in the center
            err = cx - w/2
            self.twist.linear.x = 2
            self.twist.angular.z = -float(err) / 500
            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("image", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()