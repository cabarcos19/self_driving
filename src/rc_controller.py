#!/usr/bin/python
import maestro,rospy,time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class ServoConvert():
    def __init__(self,channel,center_value=6000, range=1500):
        self.channel    = channel
	self.value      = 0.0
        self.value_out  = center_value
        self._center    = center_value
        self._range     = range
        self._half_range= 0.5*range

        #--- Convert its range in [-1, 1]
        self._sf        = 1.0/self._half_range

    def get_value_out(self, value_in):
        #--- value is in [-1, 1]
        self.value      = value_in
        self.value_out  = int(value_in*self._half_range + self._center)
        print self.channel, self.value_out
        return(self.value_out)

class CarControl():
	#bucle para usar keyboard_teleop y configurar los parametros de setTarget()

	def __init__(self):
		rospy.loginfo("Setting Up the Node...")
		rospy.init_node('car_control')
		
		self.servo = maestro.Controller()
       		self.servo.setRange(0,3000,9000)
		self.servo.setRange(1,3000,9000)
		#self.servo.setTarget(0,6000) #center position
		#Dictionary of ServoConvert
		self.actuators = {}
		self.actuators['steering'] = ServoConvert(channel=0) #servo steering channel
		self.actuators['throttle'] = ServoConvert(channel=1) #servo throttle channel

		#--- Create the Subscriber to Twist commands
        	self.ros_sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        	rospy.loginfo("> Subscriber corrrectly initialized")
	
		
       		#--- Get the last time e got a commands
        	self._last_time_cmd_rcv     = time.time()
        	self._timeout_s             = 20

        	rospy.loginfo("Initialization complete")
	
	def set_actuators_from_cmdvel(self, message):
        	"""
        	Get a message from cmd_vel, assuming a maximum input of 1
       		"""
               
                pub_steering = rospy.Publisher('/pwm_steering', Int32, queue_size=10)
                pub_throttle = rospy.Publisher('/pwm_throttle', Int32, queue_size=10)
        	#-- Save the time
        	self._last_time_cmd_rcv = time.time()

        	#-- Convert vel into servo values
        	th = self.actuators['throttle'].get_value_out(message.linear.x)
        	st = self.actuators['steering'].get_value_out(message.angular.z)
        	#rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(message.linear.x, message.angular.z))
        	#rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(th,st))
		#Sending info to the servos 
		self.servo.setTarget(0,st)
		self.servo.setTarget(1,th)
                pub_steering.publish(st)
                pub_throttle.publish(th)    
	def set_actuators_idle(self):
        	#-- Convert vel into servo values
        	th = self.actuators['throttle'].get_value_out(0)
       		st = self.actuators['steering'].get_value_out(0)
        	rospy.loginfo("Setting actutors to idle")
		self.servo.setTarget(0,st)
		self.servo.setTarget(1,th)

    	@property
    	def is_controller_connected(self):
        	#print time.time() - self._last_time_cmd_rcv
        	return(time.time() - self._last_time_cmd_rcv < self._timeout_s)



    	def run(self):
        	#--- Set the control rate
        	rate = rospy.Rate(10)

        	while not rospy.is_shutdown():
            		#print self._last_time_cmd_rcv, self.is_controller_connected
            		if not self.is_controller_connected:
                		self.set_actuators_idle()

            		rate.sleep()




if __name__ == "__main__":
    rc_control     = CarControl()
    rc_control.run()

