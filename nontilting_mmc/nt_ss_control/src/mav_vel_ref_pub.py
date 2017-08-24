#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Pose, Vector3
import dynamic_reconfigure.client
from mav_msgs.cfg import MavAttitudeCtlParamsConfig
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import math
import numpy

class ReferencePublisher:

	def __init__(self):
		self.time_old = rospy.Time.now()
		self.pose_old = Pose()
		self.start_flag = False
		self.vel_ref_msg = Vector3()
		self.position = Vector3()
		self.t_ramp = 4
		self.speed = 0.5
		self.t_st = 10
		self.dir = 1
		self.vel_ref_pub = rospy.Publisher('/arducopter/vel_ref', Vector3, queue_size=5)
		self.position_pub = rospy.Publisher('/arducopter/pos_ref', Vector3, queue_size=5)
		#self.client = dynamic_reconfigure.client.Client('/mass_ctl_attitude', timeout=30, config_callback=self.reconf_cb)
		rospy.sleep(0.1)

	def run(self):
		rate = rospy.Rate(10.0) # 7 sec
		i = 8
		self.position.x = 0
		self.position.y = 0
		self.position.z = 2
		self.position_pub.publish(self.position)
		self.position_pub.publish(self.position)
		self.position_pub.publish(self.position)
		rospy.sleep(1) #wait to climb to height 
		rospy.sleep(1)
		time_start = rospy.Time.now()
    
		while not rospy.is_shutdown():
	
			if(self.dir==1):
				self.speed = 0.5
			elif(self.dir == 0):
				self.speed = 0.0
			elif(self.dir == -1):
				self.speed = -0.5
	
			now = rospy.Time.now()	
			if(now.to_sec() - time_start.to_sec())<= self.t_ramp:
				self.vel_ref_msg.x = self.speed/self.t_ramp*(now.to_sec()-time_start.to_sec())   
			elif (now.to_sec()-time_start.to_sec())<=self.t_ramp+self.t_st:
				self.vel_ref_msg.x = self.speed
		   	elif (now.to_sec()-time_start.to_sec())<=2*self.t_ramp+self.t_st:
		   		self.vel_ref_msg.x = self.speed-self.speed/self.t_ramp*(now.to_sec()-time_start.to_sec()-(self.t_ramp+self.t_st))
		   	else:
		   		self.vel_ref_msg.x = 0
		   		self.dir = self.dir-1
		   		time_start = rospy.Time.now()
		   	
	
			self.vel_ref_msg.y = 0
			self.vel_ref_msg.z = 0
			self.vel_ref_pub.publish(self.vel_ref_msg)
			rate.sleep()
			if (self.dir < -1):
				self.vel_ref_msg.y = 0
				self.vel_ref_msg.z = 0
				self.vel_ref_msg.x = 0 		      
				break

		
if __name__ == '__main__':

    rospy.init_node('mav_vel_ref_publisher')
    ref_pub = ReferencePublisher()
    print "Starting publishing velocity reference"
    ref_pub.run()
    print "Stopped publishing velocity reference"
