#!/usr/bin/env python
import roslib; roslib.load_manifest('project2')
import rospy
import rospkg
import sys

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from blobfinder2.msg import MultiBlobInfo3D

from transform2d import transform2d_from_ros_transform, distance_2d, Transform2D

import tf
import math
import numpy as np

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# minimum duration of safety stop (s)
STOP_DURATION = rospy.Duration(1.0)

# minimum distance of cones to make gate (m)
MIN_GATE_DIST = 0.75

# maximum distance of cones to make gate (m)
MAX_GATE_DIST = 1.25

# minimum number of pixels to identify cone (px)
MIN_CONE_AREA = 200

# maximum change in linear velocity per 0.01s timestep
MAX_LINEAR_CHANGE = 0.001 # TODO: set to reasonable value

# maximum change in angular velocity per 0.01s timestep
MAX_ANGULAR_CHANGE = 0.01 # TODO: set to reasonable value

cur_row = 0

######################################################################
# returns lo, if x < lo; hi, if x > hi, or x otherwise

def clamp(x, lo, hi):
    return max(lo, min(x, hi))

######################################################################
# should return desired_vel if absolute difference from prev_vel is
# less than max_change, otherwise returns prev_vel +/- max_change,
# whichever is closer to desired_vel.

def filter_vel(prev_vel, desired_vel, max_change):
    # TODO: writeme - you may find clamp() above helpful
    abs_diff = abs(prev_vel - desired_vel)
    if abs_diff < max_change:
        return desired_vel
    else:
        prev_plus = abs(prev_vel + max_change - desired_vel)
        prev_minus = abs(prev_vel - max_change - desired_vel)
        if prev_plus < prev_minus:
            return prev_vel + max_change
        else:
            return prev_vel - max_change
    return desired_vel

######################################################################
# read a course from a text description and return a list of string
# triples like ('left', 'green', 'yellow')

def read_course(course_file):

    VALID_ACTIONS = ['left', 'right', 'tape']
    VALID_COLORS = ['yellow', 'green']

    input_stream = open(course_file, 'r')

    rows = []

    for line in input_stream:

        tokens = line.lower().split()

        if len(tokens) != 3:
            rospy.logerr('syntax error in ' + course_file)
            sys.exit(1)

        action, lcolor, rcolor = tokens

        if action not in VALID_ACTIONS:
            rospy.logerr('unexpected action type ' + action + ' in ' + course_file)
            sys.exit(1)

        if lcolor not in VALID_COLORS or rcolor not in VALID_COLORS:
            rospy.logerr('unexpected color pair {} {} in {}'.format(
                lcolor, rcolor, course_file))

        row = (action, lcolor, rcolor)
        rows.append(row)

    return rows

######################################################################
# define a class to handle our simple controller

class Controller:

    # initialize our controller
    def __init__(self):

        # initialize our ROS node
        rospy.init_node('starter')

        # read the course file and store it into self.course
        self.setup_course()

        # set up publisher for commanded velocity
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                                           Twist, queue_size=10)

        # set up a TransformListener to get odometry information
        self.odom_listener = tf.TransformListener()

        # record whether we should stop for safety
        self.should_stop = 0
        self.time_of_stop = rospy.get_rostime() - STOP_DURATION

        # initialize a dictionary of two empty lists of cones. each
        # list should hold world-frame locations of cone in XY
        self.cone_locations = dict(yellow=[], green=[])

        # this bool will be set to True when we get a new cone message
        self.cones_updated = False

        # No gate yet
        self.cur_gate = None

        # pre action means you're turning left, right, or following the tape
        self.pre_act = True

        # Start at 0 velocity
        self.prev_cmd_vel = Twist()

        # set up our controller
        rospy.Timer(CONTROL_PERIOD,
                    self.control_callback)

        # set up subscriber for sensor state for bumpers/cliffs
        rospy.Subscriber('/mobile_base/sensors/core',
                         SensorState, self.sensor_callback)

        # Get blobs
        rospy.Subscriber('/blobfinder2/blobs3d',
                         MultiBlobInfo3D, self.blob_callback)

        # passed gate
        self.passed = False

        # start with action from the first row of self.course
        # self.cur_action = 0

        # x position of the tape to follow
        self.blob_center = None

    # set up the course from the file
    def setup_course(self):
        # read in the course file and store it into self.course
        args = rospy.myargv(argv=sys.argv)

        if len(args) == 1:
            course_file = 'course.txt'
        elif len(args) == 2:
            course_file = args[1]
        else:
            rospy.logerr('expected 0 or 1 command-line arguments')
            sys.exit(1)

        if not course_file.startswith('/'):
            rospack = rospkg.RosPack()
            project2_path = rospack.get_path('project3')
            course_file = project2_path + '/data/' + course_file

        self.course = read_course(course_file)

        rospy.loginfo('read course: '+repr(self.course))

    # called when sensor msgs received - just copy sensor readings to
    # class member variables
    def sensor_callback(self, msg):

        if msg.bumper & SensorState.BUMPER_LEFT:
            rospy.loginfo('***LEFT BUMPER***')
        if msg.bumper & SensorState.BUMPER_CENTRE:
            rospy.loginfo('***MIDDLE BUMPER***')
        if msg.bumper & SensorState.BUMPER_RIGHT:
            rospy.loginfo('***RIGHT BUMPER***')
        if msg.cliff:
            rospy.loginfo('***CLIFF***')

        if msg.bumper or msg.cliff:
            self.should_stop = True
            self.time_of_stop = rospy.get_rostime()
        else:
            self.should_stop = False

    # called when a blob message comes in
    def blob_callback(self, msg):
        num = len(msg.color_blobs)
        for i in range(num):
            color_blob = msg.color_blobs[i]
            color = color_blob.color.data
            if color == 'red_tape':
                self.tape_callback(color_blob)
            elif color == 'yellow_cone':
                self.cone_callback(color_blob, 'yellow')
            elif color == 'green_cone':
                self.cone_callback(color_blob, 'green')
            elif color == 'blue_ball':
                pass # This branch shouldn't be needed


    # called when a ball blob message comes in
    def tape_callback(self, bloblist):
        # TODO: Use Project 1 Code here
        if len(bloblist.blobs) != 0:
            largest = bloblist.blobs[0]
            for i in range(len(bloblist.blobs)):
                if bloblist.blobs[i].blob.area > largest.blob.area:
                    largest = bloblist.blobs[i]
            self.blob_center = largest.blob.cx
        return

    # called when a cone related blob message comes in
    def cone_callback(self, bloblist, color):

        T_world_from_robot = self.get_current_pose()
        if T_world_from_robot is None:
            rospy.logwarn('no xform yet in blobs3d_callback')
            return

        blob_locations = []

        for blob3d in bloblist.blobs:
            if blob3d.have_pos and blob3d.blob.area > MIN_CONE_AREA and blob3d.blob.cy > 240:
                blob_in_robot_frame = np.array([blob3d.position.z, -blob3d.position.x])
                blob_dir = blob_in_robot_frame / np.linalg.norm(blob_in_robot_frame)
                blob_in_robot_frame += blob_dir * 0.04 # offset radius
                blob_locations.append( T_world_from_robot * blob_in_robot_frame )

        self.cone_locations[color] = blob_locations
        self.cones_updated = True


    # get current pose from TransformListener
    def get_current_pose(self):

        try:

            ros_xform = self.odom_listener.lookupTransform(
                '/odom', '/base_footprint',
                rospy.Time(0))

        except tf.LookupException:

            return None

        return transform2d_from_ros_transform(ros_xform)

    # called when we need to find a gate
    def find_gates(self, cur_pose):

        # build a list of all cones by combining colors
        all_cones = []

        for color in ['yellow', 'green']:
            for world_xy in self.cone_locations[color]:
                all_cones.append( (color, world_xy) )

        # get the inverse transformation of cur pose to be able to map
        # points back into robot frame (so we can assess which gates
        # are left and right)
        T_robot_from_world = cur_pose.inverse()

        # build a list of all gates by investigating pairs of cones
        all_gates = []

        # for each cone
        for i in range(len(all_cones)):

            # get color, world pos, robot pos
            (ci, wxy_i) = all_cones[i]
            rxy_i = T_robot_from_world * wxy_i

            # for each other cone
            for j in range(i):

                # get color, world pos, robot pos
                (cj, wxy_j) = all_cones[j]
                rxy_j = T_robot_from_world * wxy_j

                # get distance between cone pair
                dist_ij = distance_2d(wxy_i, wxy_j)

                # if in the range that seems reasonable for a gate:
                if dist_ij > MIN_GATE_DIST and dist_ij < MAX_GATE_DIST:

                    # get midpoint of cones
                    midpoint = 0.5 * (wxy_i + wxy_j)

                    # sort the cones left to right (we want left to
                    # have a larger Y coordinate in the robot frame)
                    if rxy_i[1] > rxy_j[1]:
                        lcolor, wxy_l, rcolor, wxy_r = ci, wxy_i, cj, wxy_j
                    else:
                        lcolor, wxy_l, rcolor, wxy_r = cj, wxy_j, ci, wxy_i

                    # the direction of the gate frame's local Y axis
                    # is obtained by subtracting the right cone from
                    # the left cone.
                    gate_ydir = (wxy_l - wxy_r)

                    # the angle of rotation for the gate is obtained
                    # by the arctangent given here
                    gate_theta = math.atan2(-gate_ydir[0], gate_ydir[1])

                    T_world_from_gate = Transform2D(midpoint[0], midpoint[1],
                                                   gate_theta)

                    #rospy.loginfo('found {0},{1} gate with T_world_from_gate = {2}'.format(
                    #        lcolor, rcolor, T_world_from_gate))

                    gate = (T_world_from_gate, lcolor, rcolor, wxy_l, wxy_r, dist_ij)

                    all_gates.append( gate )

        self.cones_updated = False

        return all_gates

    def drive_towards(self,point,dist):
        cmd_vel = Twist()
        beta = math.atan2(point[1],point[0])
        if dist > 0.1 and self.passed == False:
            cmd_vel.linear.x = 0.1*dist + 0.2
            cmd_vel.angular.z = 0.3*beta + 0.3*np.sign(beta)
            rospy.loginfo('driving to point {}'.format(point))
        #if point[0] > 0.1:
        #cmd_vel.angular.z = 0.1*beta + 0.1*np.sign(beta)
        else:
            cmd_vel.angular.z = 0
            cmd_vel.linear.x = 0.0
            self.passed = True
            self.pre_act = True
            global cur_row
            cur_row += 1
        return cmd_vel

    def check_gates(self):
        for gate in self.find_gates(self.get_current_pose()):
            if gate[1] == self.course[cur_row][1]:
                if gate[2] == self.course[cur_row][2]:
                    self.pre_act = False
                    return gate
        return None

    # called periodically to do top-level coordination of behaviors
    def control_callback(self, timer_event=None):

        # initialize vel to 0, 0
        cmd_vel = Twist()

        time_since_stop = rospy.get_rostime() - self.time_of_stop

        cur_pose = self.get_current_pose()

        if self.should_stop or time_since_stop < STOP_DURATION:

            rospy.loginfo('stopped')

        else: # not stopped for safety

            # TODO: For testing, hard-code a commanded velocity here to
            # test & verify filter_vel, then remove when you are
            # happy with filtering behavior (same as project 2)

            if self.cones_updated and cur_pose is not None:
                all_gates = self.find_gates(cur_pose)

                # now update self.cur_gate from gate list
                if len(all_gates):
                    # TODO: something more sophisticated than grabbing
                    # the first gate off the list!  Make sure to find
                    # the gate matching the colors we are looking for,
                    # and near the one we have already recorded if we have
                    # seen a gate so far...
                    if self.pre_act == True:
                        self.cur_gate = self.check_gates()
                    if self.cur_gate is None:
                        rospy.loginfo('waiting for gate...')
                    else:
                        rospy.loginfo('updated current gate!')

            if self.pre_act == True:
                self.passed = False
                if self.course[cur_row][0] == 'left':
                    rospy.loginfo('turning left...')
                    cmd_vel.angular.z = 1.0
                    self.cur_gate = None
                elif self.course[cur_row][0] == 'right':
                    rospy.loginfo('turning right...')
                    cmd_vel.angular.z = -1.0
                    self.cur_gate = None
                else:
                    # add code to follow the line here
                    cmd_vel.linear.x = 0.3
                    cmd_vel.angular.z = 0.004 * (360 - self.blob_center)
                    rospy.loginfo('following the tape')
                    self.cur_gate = None
            elif self.cur_gate is not None:

                T_world_from_robot = cur_pose   # .translation() robot in world coords
                T_world_from_gate = self.cur_gate[0]    # .translation() gate in world coords

                dist_to_gate = distance_2d(T_world_from_robot.translation(),
                                           T_world_from_gate.translation())

                rospy.loginfo('current gate is {}-{} at {} in world coords, {} meters away'.format(
                    self.cur_gate[1], self.cur_gate[2],
                    T_world_from_gate.translation(),
                    dist_to_gate))

                # TODO: task 1 - set cmd_vel based on pure pursuit
                # strategy in order to traverse gate

                alpha = 0.3
                T_gate_from_world = T_world_from_gate.inverse()
                robot_in_gate = T_gate_from_world*T_world_from_robot
                dest_in_gate = robot_in_gate
                dest_in_gate.y = 0
                dest_in_gate.x += alpha
                dest_in_world = T_world_from_gate*dest_in_gate
                dest = T_world_from_robot.inverse() * dest_in_world # dest in robot

                #if dist_to_gate < 0.1:
                #       self.should_stop = 1
                if self.should_stop == 0:
                        vel = self.drive_towards(dest.translation(),dist_to_gate)
                        cmd_vel = vel



            # now filter large changes in velocity before commanding
            # robot - note we don't filter when stopped
            cmd_vel.linear.x = filter_vel(self.prev_cmd_vel.linear.x,
                                          cmd_vel.linear.x,
                                          MAX_LINEAR_CHANGE)

            cmd_vel.angular.z = filter_vel(self.prev_cmd_vel.angular.z,
                                           cmd_vel.angular.z,
                                           MAX_ANGULAR_CHANGE)

        self.cmd_vel_pub.publish(cmd_vel)
        self.prev_cmd_vel = cmd_vel

    # called by main function below (after init)
    def run(self):

        # timers and callbacks are already set up, so just spin
        rospy.spin()

        # if spin returns we were interrupted by Ctrl+C or shutdown
        rospy.loginfo('goodbye')


# main function
if __name__ == '__main__':
    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass

                                                                                                                                                                                                                                                                                                                                               
