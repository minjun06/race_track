#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from math import pi

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.state = "PATROLLING"
        self.logcount = 0
        self.lostcount = 0
        self.head_range = 4
        # turning time for obstacle avoidance
        self.turning_time = None
        self.turning_start_time = None
        # waiting time for potential moving obstacle
        # wait for 5 sec if the obstacle moves away keep following, otherwise we escape the obstacle by turning 90 degree to its position
        self.waiting_time = rospy.Duration(secs=5)
        self.waiting_start_time = None

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
        resized_mask = cv2.resize(mask, (480,270))
        cv2.imshow("band", resized_mask)
        print "pixel value: " + str(hsv[7*h/8, w/2])
        # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))
        if self.state != "OBSTACLE_DETECTED" and self.state != "WAITING":
            if M['m00'] > 0:
                self.state = "FOLLOWING_LINE"
                cx = int(M['m10']/M['m00']) - 300
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                # add a turn if the centroid is not in the center
                err = cx - w/2
                self.twist.linear.x = 2
                self.twist.angular.z = -float(err) / 500
                self.cmd_vel_pub.publish(self.twist)
            else: #patrolling
                self.patrol()
        resized = cv2.resize(image, (480,270))
        cv2.imshow("image", resized)
        cv2.waitKey(3)

    # This method simply let the robot to patrol in straight line
    def patrol(self):
        self.state = "PATROLLING"
        self.twist.linear.x = 1
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def scan_callback(self, msg):
        ranges = numpy.array(self.filter(msg))
        # head area for the robot
        head = ranges[342:360] + ranges[0:18] 
        # min_value for the range to filter the noise
        self.head_range = numpy.amin(head)
        print(self.head_range)
        if self.turning_time == None:#not in turning state
            if self.head_range < 4:
                # wait for the obstacle to move away when following
                if (self.state == "FOLLOWING_LINE" and self.waiting_start_time == None) or self.waiting_start_time != None:
                    self.wait()
                else:
                    # actual angle = index(head) - 18
                    self.start_turn(ranges, numpy.argmin(head) - 18)
            elif self.state != "FOLLOWING_LINE" and self.state != "TURNING_BACK":
                if self.state == "WAITING":# obstacle move away
                    self.waiting_start_time = None
                self.patrol()
        else:
            self.turn()
    
    # This method deals with the WAITING state of the robot
    def wait(self):
        self.state = "WAITING"
        if self.waiting_start_time == None:
            self.waiting_start_time = rospy.Time.now()
        if rospy.Time.now() - self.waiting_start_time < self.waiting_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
        else:
            #end waiting
            self.waiting_start_time = None
            self.state = "OBSTACLE_DETECTED"
    

    # This method start the turning process when the robot is aiming to skip the obstacle
    def start_turn(self, ranges, min_angle):
        self.state = "OBSTACLE_DETECTED"
        self.turning_start_time = rospy.Time.now()
        # turning orientation
        time_duration = 0
        if ranges[(min_angle - 90 + 360)%360] > ranges[min_angle + 90]:
            self.turn_anti_clockwise = -1
            time_duration = float(90 - min_angle)/(180/10)
        else:
            self.turn_anti_clockwise = 1
            time_duration = float(min_angle + 90)/(180/10)
        self.turning_time = rospy.Duration(secs=time_duration)
    
     # This method periodically send turning message to robot when turning time is not zero
    def turn(self):
        if rospy.Time.now() - self.turning_start_time < self.turning_time:
            self.twist.angular.z = self.turn_anti_clockwise * pi/10
            self.twist.linear.x = 0
            self.cmd_vel_pub.publish(self.twist)
        else:
            self.turning_time = None
            self.patrol()

    # This method filters the noice in the range data
    def filter(self, msg):
        ranges = list(msg.ranges)
        for i in range(len(msg.ranges)):
            if ranges[i] < msg.range_min:
                ranges[i] = msg.range_min
            elif ranges[i] > msg.range_max:
                ranges[i] = msg.range_max
        return ranges

rospy.init_node('follower')
follower = Follower()
rospy.spin()