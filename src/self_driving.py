#! /usr/bin/env python

import rospy,time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'straight',
    1: 'turn fleft',
    2: 'turn fright',
    3: 'turn right',
    4: 'turn left',
    5: 'emergency stop',
}


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:109]), 10),
        'fright': min(min(msg.ranges[110:174]), 10),
        'front':  min(min(msg.ranges[175:184]), 10),
        'fleft':  min(min(msg.ranges[185:249]), 10),
        'left':   min(min(msg.ranges[250:359]), 10),
    }

    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d_detection = 1.5
    d = 1
    d_emergency = 0.1

    if regions['front'] >= d_detection:
        state_description = 'case 0 - go straight'
        change_state(0)
    else:

        # fleft && fright > d
        if regions['fleft'] > d and regions['fright'] > d and regions['fleft'] > regions['fright']:
	    state_description = 'case x move to the fleft'
            change_state(1) #move to fleft
    
        elif regions['fleft'] > d and regions['fright'] > d and regions['fleft'] < regions['fright']:
            state_description = 'case x move to the fright'
            change_state(2) #move to fright

        # fleft && fright < d
        elif regions['fleft'] < d and regions['fright'] < d and regions['fleft'] > regions['fright']:
            state_description = 'case x move to the fleft'
            change_state(1) #move to fleft
            
        elif regions['fleft'] < d and regions['fright'] < d and regions['fleft'] < regions['fright']:
            state_description = 'case x move to the fright'
            change_state(2) #move to fright

        # fleft < d && fright > d or fleft > d && fright < d
        elif regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 2 - go front right'
            change_state(2)

        elif regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 1 - go front left'
            change_state(1)

    #emergency stop        
    if regions['front'] < d_emergency and regions['fleft'] < d_emergency and regions['fright'] < d_emergency:
  	    state_description = 'case 5 - emergency stop'
	    change_state(5)

    rospy.loginfo(regions)
   

def turn_fright():
    global regions_

    msg = Twist()
    msg.linear.x = 0.15
    msg.angular.z = -1
    return msg

def turn_right():
    global regions_

    msg = Twist()
    msg.angular.z = -1
    return msg

def turn_fleft():
    global regions_

    msg = Twist()
    msg.linear.x = 0.15
    msg.angular.z = 1
    return msg

def turn_left():
    global regions_

    msg = Twist()
    msg.angular.z = 1
    return msg

def straight():
    global regions_

    msg = Twist()
    msg.linear.x = 0.15
    return msg


def emergency_stop():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    return msg

def main():
    global pub_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = straight()
        elif state_ == 1:
            msg = turn_fleft()
        elif state_ == 2:
            msg = turn_fright()
        elif state_ == 3:
	    msg = turn_right()
	elif state_ == 4:
	    msg = turn_left()
	elif state_ == 5:
	    msg = emergency_stop()
	    pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()

