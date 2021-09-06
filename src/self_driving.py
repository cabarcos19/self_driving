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

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d_detection = 2.5
    d = 2.0
    d_emergency = 1.0

	
    if regions['front'] >= d_detection:
        #center the car
    	#obtain lateral distances
        l_left = round(regions['lateral_left'],2)
        l_right = round(regions['lateral_right'],2)
    	#calculate circuit width and center
    	circuit_width = l_left + l_right
    	center = round((l_left + l_right) / 2,2)
    	#set the limit
    	limit = round(center - circuit_width * 0.1,2)
    	rospy.loginfo('circuit with:%s ',circuit_width)
    	rospy.loginfo('center:%s ',center)
    	rospy.loginfo('limit:%s ',limit)

    	if(l_left < center):
            if(l_left < limit):
            	change_state(2)
            	print 'centering to the right'
    	elif(l_right < center):
            if(l_right < limit):
            	change_state(1)
                print 'centering to the left'
	state_description = 'case 0 - go straight'
        change_state(0)
    
    #emergency stop
    elif regions['fleft'] < d_emergency and regions['fright'] < d_emergency and regions['front'] < d_emergency :
        state_description = 'case 0: emergency stop'
        print (state_description)
        change_state(5)
    # fleft && fright > d
    elif regions['fleft'] > d and regions['fright'] > d and regions['fleft'] > regions['fright']:
	state_description = 'case 1 fleft && fright and fleft > fright move to the fleft'
        print (state_description)
        change_state(1) #move to fleft
    
    elif regions['fleft'] > d and regions['fright'] > d and regions['fleft'] < regions['fright']:
        state_description = 'case 2 fleft && fright and fleft < fright move to the fright'
        print (state_description)
        change_state(2) #move to fright

    # fleft && fright < d
    elif regions['fleft'] < d and regions['fright'] < d and regions['fleft'] > regions['fright']:
        state_description = 'case 3 fleft && fright < d and fleft > fright move to the fleft'
        print (state_description)
        change_state(4) #move to left
            
    elif regions['fleft'] < d and regions['fright'] < d and regions['fleft'] < regions['fright']:
        state_description = 'case 4 fleft && fright < d and fleft < fright move to the fright'
        print (state_description)
        change_state(3) #move to right

    # fleft < d && fright > d or fleft > d && fright < d
    elif regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 5: fleft < d && fright > d move to fright'
        print (state_description)
        change_state(2)

    elif regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 6: fleft > d && fright < d move to fleft'
        print (state_description)
        change_state(1)
   

def turn_fright():
    global regions_

    msg = Twist()
    msg.linear.x = 0.60
    msg.angular.z = -0.75
    return msg

def turn_right():
    global regions_

    msg = Twist()
    msg.linear.x = 0.55
    msg.angular.z = -1
    return msg

def turn_fleft():
    global regions_

    msg = Twist()
    msg.linear.x = 0.60
    msg.angular.z = 0.75
    return msg

def turn_left():
    global regions_

    msg = Twist()
    msg.linear.x = 0.55
    msg.angular.z = 1
    return msg

def straight():
    global regions_

    msg = Twist()
    msg.linear.x = 0.65
    return msg


def emergency_stop():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    return msg


    

def main():
    global pub_

    rospy.init_node('reading_laser')
	#queue size = 1 by default
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

