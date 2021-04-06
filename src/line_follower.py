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
        self.head_range = 3.5
        # turning time for obstacle avoidance
        self.turning_time = None
        self.turning_start_time = None
        # waiting time for potential moving obstacle
        # wait for 0 sec if the obstacle moves away keep following, otherwise we escape the obstacle by turning 90 degree to its position
        self.waiting_time = rospy.Duration(secs=0)
        self.waiting_start_time = None

    def image_callback(self, msg):
        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)

        # filter out everything that's not gray
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_gray = numpy.array([ 5, 5, 50])
        upper_gray = numpy.array([ 40, 40, 80])
        mask = cv2.inRange(hsv,  lower_gray, upper_gray)
        masked = cv2.bitwise_and(image, image, mask=mask)
        
        # image ROI (Region of Interest)
        h, w, d = image.shape
        search_top = h / 2
        search_bot = search_top + 200
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        resized_mask = cv2.resize(mask, (480,270))
        cv2.imshow("band", resized_mask)
        
        # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        self.logcount += 1

        # if we are following the line
        if self.state != "OBSTACLE_DETECTED" and self.state != "WAITING":
            # if there is road to follow
            if M['m00'] > 0:
                self.state = "FOLLOWING_LINE"
                cx = int(M['m10']/M['m00']) - 50
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                # add a turn if the centroid is not in the center
                err = cx - w/2
                self.twist.linear.x = 1.5
                self.twist.angular.z = -float(err) / 375
                self.cmd_vel_pub.publish(self.twist)
            else: #patrolling
                self.patrol()
        print "state: " + self.state
        resized = cv2.resize(image, (480,270))
        cv2.imshow("image", resized)
        cv2.waitKey(3)

    # This method simply let the robot to patrol in straight line
    def patrol(self):
        self.state = "PATROLLING"
        self.twist.linear.x = 1
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    # This method reads scan data to identify obstacles
    def scan_callback(self, msg):
        ranges = self.filter(msg)
        # head area for the robot
        head = numpy.concatenate((numpy.array(ranges[350:360]), numpy.array(ranges[0:10])))
        # min_value for the range to filter the noise
        self.head_range = numpy.amin(head)
        print(self.head_range)
        if self.turning_time == None:#not in turning state
            if self.head_range < 3:
                # wait for the obstacle to move away when following
                if (self.state == "FOLLOWING_LINE" and self.waiting_start_time == None) or self.waiting_start_time != None:
                    self.wait()
                else:
                    self.start_turn(ranges, 10)
            elif self.state != "FOLLOWING_LINE":
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
    

    # This method start the turning process when the robot is aiming to avoid the obstacle
    def start_turn(self, ranges, head_range):
        self.state = "OBSTACLE_DETECTED"
        self.turning_start_time = rospy.Time.now()
        # turning orientation
        # maybe we can turing while keep detecting
        time_duration = 0
        right_angle = self.get_angle(True, ranges, head_range)
        left_angle = self.get_angle(False, ranges, head_range)
        if left_angle > right_angle:
            # turn right
            print("turning right")
            self.turn_anti_clockwise = -1
            time_duration = float(right_angle)/(180/4)
        else: # turn left
            print("turning left")
            self.turn_anti_clockwise = 1
            time_duration = float(left_angle)/(180/4)
        self.turning_time = rospy.Duration(secs=time_duration)
    
    # this method check the outmost boundary of the obstacle
    def get_angle(self, right, ranges, head_range):
        if right:
            #find the space in the right region
            for i in range(90):
                if ranges[359 - i] >= 3.5:
                    # ten more angle is added for safety
                    return head_range + i + 20
        else:
            #find the space in the left region
            for i in range(90):
                if ranges[i] >= 3.5:
                    return head_range + i + 20
        return 0

    # This method periodically send turning message to robot when turning time is not zero
    def turn(self):
        if rospy.Time.now() - self.turning_start_time < self.turning_time:
            self.twist.angular.z = self.turn_anti_clockwise * pi/4
            self.twist.linear.x = 0.5
            self.cmd_vel_pub.publish(self.twist)
        else:
            self.turning_time = None
            self.patrol()

    # This method filters the noice in the range data
    # max = 3.5
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