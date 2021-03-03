#! /usr/bin/env python

import rospy,time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

pub_ = None
regions_ = {
    'back_right': 0,
    'lateral_right':0,
	'middle_right':0,
	'fright': 0,
    'front': 0,
    'fleft': 0,
	'middle_left': 0,
	'lateral_left': 0,
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
        'back_right':  min(min(msg.ranges[0:84]), 12),
		'lateral_right': min(min(msg.ranges[85:94]),12),
        'middle_right': min(min(msg.ranges[95:109]),12),
		'fright': min(min(msg.ranges[110:174]), 12),
        'front':  min(min(msg.ranges[175:184]), 12),
        'fleft':  min(min(msg.ranges[185:249]), 12),
		'middle_left': min(min(msg.ranges[250:264]),12),
        'lateral_left': min(min(msg.ranges[265:274]),12),
		'back_left':   min(min(msg.ranges[275:359]), 12),
    }
	
    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Action - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def center_car(float lateral_left_laser,float lateral_right_laser):
	l_left = round(lateral_left_laser,2)
	l_right = round(lateral_right_laser,2)
	center = l_left + l_right
	center_r = round(center,2)
	
	if(l_left < center_r):
		while(l_left != center_r):
			turn_fleft()
	else:
		while(l_right != center_r):
			turn_fright()


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
	
	center_car(regions['lateral_left'],regions['lateral_right'])
	
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
		else:
        	state_description = 'case 5 - emergency stop'
	    	change_state(5)

    #rospy.loginfo(regions)
   

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
	#queue size = 1 by default
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

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

