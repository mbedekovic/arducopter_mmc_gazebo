#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import  Server
from mav_msgs.cfg import MavCtlParamsConfig
from mav_msgs.msg import PIDController
from mav_msgs.msg import MotorSpeed
from datetime import datetime
import math

class HeightControl:
    '''
    Class implements ROS node for cascade (z, vz) PID control for MAV height.
    Subscribes to:
        /morus/pose       - used to extract z-position of the vehicle
        /morus/velocity   - used to extract vz of the vehicle
        /morus/pos_ref    - used to set the reference for z-position
        /morus/vel_ref    - used to set the reference for vz-position (useful for testing velocity controller)

    Publishes:
        /morus/mot_vel_ref  - referent value for thrust in terms of motor velocity (rad/s)
        /morus/pid_z        - publishes PID-z data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_vz        - publishes PID-vz data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controller params online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False         # indicates if we received the first measurement
        self.config_start = False       # flag indicates if the config callback is called for the first time

        self.x_sp = 0                   # x-position set point
        self.y_sp = 0                   # y-position set point
        self.z_sp = 0                   # z-position set point
        self.x_mv = 0                   # x-position measured value
        self.y_mv = 0                   # y-position measured value
        self.z_mv = 0                   # z-position measured value
        self.pid_x = PID()              # pid instance for x control
        self.pid_y = PID()              # pid instance for y control
        self.pid_z = PID()              # pid instance for z control

        self.vx_sp = 0                  # vx velocity set_point
        self.vy_sp = 0                  # vx velocity set_point
        self.vz_sp = 0                  # vz velocity set_point
        self.vx_mv = 0                  # vx velocity measured value
        self.vy_mv = 0                  # vy velocity measured value
        self.vz_mv = 0                  # vz velocity measured value
        self.vx_mv_old = 0              # vz velocity old measured value
        self.vy_mv_old = 0              # vz velocity old measured value
        self.vz_mv_old = 0              # vz velocity old measured value
        self.pid_vx = PID()             # pid instance for x-velocity control
        self.pid_vy = PID()             # pid instance for y-velocity control
        self.pid_vz = PID()             # pid instance for z-velocity control

        self.euler_mv = Vector3()           # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)    # euler angles referent values
        self.euler_rate_mv = Vector3()      # measured angular velocities

        self.yaw_sp = 0
        self.yaw_mv = 0
        self.pid_yaw = PID()
        self.pid_yaw_rate = PID()
        self.dwz = 0
        self.w0 = 923.7384

        self.dx_speed = 0
        self.dy_speed = 0 

        #########################################################
        #########################################################
        # Add parameters for z controller
        self.pid_z.set_kp(1.025)
        self.pid_z.set_ki(0.018)
        self.pid_z.set_kd(0.0)

        # Add parameters for vz controller
        self.pid_vz.set_kp(209.79)
        self.pid_vz.set_ki(0.0)
        self.pid_vz.set_kd(0.1)

        self.pid_vx.set_kp(24)
        self.pid_vx.set_ki(20)
        self.pid_vx.set_kd(12)
        self.pid_vx.set_lim_high(0.05 * self.w0)
        self.pid_vx.set_lim_low(-0.05 * self.w0)

        self.pid_vy.set_kp(24)
        self.pid_vy.set_ki(20)
        self.pid_vy.set_kd(12)
        self.pid_vy.set_lim_high(0.05 * self.w0)
        self.pid_vy.set_lim_low(-0.05 * self.w0)

        self.pid_x.set_kp(0.0)
        self.pid_x.set_ki(0.0)
        self.pid_x.set_kd(0.0)
        self.pid_x.set_lim_high(0.75)
        self.pid_x.set_lim_low(-0.75)

        self.pid_yaw.set_kp(1.86)
        self.pid_yaw.set_ki(0)
        self.pid_yaw.set_kd(0)

        self.pid_yaw_rate.set_kp(6.5)
        self.pid_yaw_rate.set_ki(0)
        self.pid_yaw_rate.set_kd(0)

        #########################################################
        #########################################################


        self.pid_z.set_lim_high(5)      # max vertical ascent speed
        self.pid_z.set_lim_low(-5)      # max vertical descent speed

        self.pid_vz.set_lim_high(1475)   # max velocity of a motor
        self.pid_vz.set_lim_low(-1475)   # min velocity of a motor

        self.mot_speed = 0              # referent motors velocity, computed by PID cascade

        self.gm_attitude_ctl = 0        # flag indicates if attitude control is turned on
        self.gm_attitude_ctl = rospy.get_param('~gm_attitude_ctl')

        self.t_old = 0

        rospy.Subscriber('/arducopter/sensors/pose1', PoseWithCovarianceStamped, self.pose_cb)
        rospy.Subscriber('/arducopter/velocity', TwistStamped, self.vel_cb)
        rospy.Subscriber('/arducopter/vel_ref', Vector3, self.vel_ref_cb)
        rospy.Subscriber('/arducopter/pos_ref', Vector3, self.pos_ref_cb)
        rospy.Subscriber('/arducopter/euler_ref', Vector3, self.euler_ref_cb)
        rospy.Subscriber('/arducopter/imu', Imu, self.ahrs_cb)


        self.pub_pid_z = rospy.Publisher('/arducopter/pid_z', PIDController, queue_size=1)
        self.pub_pid_vz = rospy.Publisher('/arducopter/pid_vz', PIDController, queue_size=1)
        self.pub_pid_x = rospy.Publisher('/arducopter/pid_x', PIDController, queue_size=1)
        self.pub_pid_vx = rospy.Publisher('/arducopter/pid_vx', PIDController, queue_size=1)
        self.pub_pid_vy = rospy.Publisher('/arducopter/pid_vy', PIDController, queue_size=1)
        self.pub_pid_yaw = rospy.Publisher('/arducopter/pid_yaw', PIDController, queue_size=1)
        self.pub_pid_wz = rospy.Publisher('/arducopter/pid_yaw_rate', PIDController, queue_size=1)
        self.mot_ref_pub = rospy.Publisher('/arducopter/mot_vel_ref', Float32, queue_size=1)
        self.pub_mot = rospy.Publisher('/arducopter/command/motors', MotorSpeed, queue_size=1)
        self.cfg_server = Server(MavCtlParamsConfig, self.cfg_callback)
        self.ros_rate = rospy.Rate(20)
        self.t_start = rospy.Time.now()

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for z and vz control.
        '''

        while not self.start_flag:
            print 'Waiting for velocity measurements.'
            rospy.sleep(0.5)
        print "Starting height control."

        self.t_old = rospy.Time.now()
        #self.t_old = datetime.now()

        while not rospy.is_shutdown():
            self.ros_rate.sleep() # 5 ms sleep

            ########################################################
            ########################################################
            # Implement cascade PID control here.

            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            #t = datetime.now()
            #dt = (t - self.t_old).total_seconds()
            #if dt > 0.105 or dt < 0.095:
            #    print dt

            self.t_old = t

            self.mot_speed_hover = 923.7384#300.755#432#527 # roughly
            #height control
            vz_ref = self.pid_z.compute(self.z_sp, self.z_mv, dt)

            self.mot_speed = self.mot_speed_hover + \
                        self.pid_vz.compute(vz_ref, self.vz_mv, dt)

            # vx control
            #vx_ref = self.pid_x.compute(self.x_sp, self.x_mv, dt)
            self.dx_speed = self.pid_vx.compute(self.vx_sp, self.vx_mv, dt)
            self.dy_speed = self.pid_vy.compute(self.vy_sp, self.vy_mv, dt)

            yaw_r_ref = self.pid_yaw.compute(self.euler_sp.z, self.euler_mv.z, dt)
            self.dwz = self.pid_yaw_rate.compute(yaw_r_ref, self.euler_rate_mv.z, dt)

            ########################################################
            ########################################################


            if self.gm_attitude_ctl == 0:

                # Publish motor velocities
                mot_speed_msg = MotorSpeed()
                #dx_speed = self.x_sp * self.mot_speed
                #dy_speed = self.y_sp * self.mot_speed
                mot_speed1 = self.mot_speed - self.dx_speed + self.dwz
                mot_speed3 = self.mot_speed + self.dx_speed + self.dwz
                mot_speed2 = self.mot_speed - self.dy_speed - self.dwz
                mot_speed4 = self.mot_speed + self.dy_speed - self.dwz
                
                if(self.z_mv < 0.14 and self.z_sp == 0):
                	mot_speed_msg.motor_speed = [0,0,0,0]
                	self.pub_mot.publish(mot_speed_msg)
                else: 
                	mot_speed_msg.motor_speed = [mot_speed1, mot_speed2, mot_speed3, mot_speed4]
                	self.pub_mot.publish(mot_speed_msg) 
            else:
                # publish referent motor velocity to attitude controller
                mot_speed_msg = Float32(self.mot_speed)
                self.mot_ref_pub.publish(mot_speed_msg)

            self.ros_rate.sleep()  # 5 ms sleep


            # Publish PID data - could be useful for tuning
            self.pub_pid_z.publish(self.pid_z.create_msg())
            self.pub_pid_vz.publish(self.pid_vz.create_msg())

            #self.pub_pid_x.publish(self.pid_x.create_msg())
            self.pub_pid_vx.publish(self.pid_vx.create_msg())
            self.pub_pid_vy.publish(self.pid_vy.create_msg())

            self.pub_pid_yaw.publish(self.pid_yaw.create_msg())
            self.pub_pid_wz.publish(self.pid_yaw_rate.create_msg())

    def pose_cb(self, msg):
        '''
        Pose (6DOF - position and orientation) callback.
        :param msg: Type PoseWithCovarianceStamped
        '''
        self.x_mv = msg.pose.pose.position.x
        self.y_mv = msg.pose.pose.position.y
        self.z_mv = msg.pose.pose.position.z

    def vel_cb(self, msg):
        '''
        Velocity callback (linear velocity - x,y,z)
        :param msg: Type Vector3Stamped
        '''
        if not self.start_flag:
            self.start_flag = True
        a = 0.99
        self.vx_mv = (1-a)*msg.twist.linear.x + a * self.vx_mv_old
        self.vx_mv_old = self.vx_mv
        self.vy_mv = (1-a)*msg.twist.linear.y + a * self.vy_mv_old
        self.vy_mv_old = self.vy_mv
        self.vz_mv = (1-a)*msg.twist.linear.z + a * self.vz_mv_old
        self.vz_mv_old = self.vz_mv

    def vel_ref_cb(self, msg):
        '''
        Referent velocity callback. Use received velocity values when during initial tuning
        velocity controller (i.e. when position controller still not implemented).
        :param msg: Type Vector3
        '''
        self.vx_sp = msg.x
        self.vy_sp = msg.y
        #self.vz_sp = msg.z

    def pos_ref_cb(self, msg):
        '''
        Referent position callback. Received value (z-component) is used as a referent height.
        :param msg: Type Vector3
        '''
        self.x_sp = msg.x
        self.y_sp = msg.y
        self.z_sp = msg.z

    def ahrs_cb(self, msg):
        '''
        AHRS callback. Used to extract roll, pitch, yaw and their rates.
        We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type sensor_msgs/Imu
        '''
        if not self.start_flag:
            self.start_flag = True

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z

        sx = math.sin(self.euler_mv.x)   # sin(roll)
        cx = math.cos(self.euler_mv.x)   # cos(roll)
        cy = math.cos(self.euler_mv.y)   # cos(pitch)
        ty = math.tan(self.euler_mv.y)   # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_mv.x = p + sx * ty * q + cx * ty * r
        self.euler_rate_mv.y = cx * q - sx * r
        self.euler_rate_mv.z = sx / cy * q + cx / cy * r

    def euler_ref_cb(self, msg):
        '''
        Euler ref values callback.
        :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
        '''
        self.euler_sp = msg

    def cfg_callback(self, config, level):
        """
        Callback for dynamically reconfigurable parameters (P,I,D gains for height and velocity controller)
        """

        if not self.config_start:
            # callback is called for the first time. Use this to set the new params to the config server
            config.z_kp = self.pid_z.get_kp()
            config.z_ki = self.pid_z.get_ki()
            config.z_kd = self.pid_z.get_kd()

            config.vz_kp = self.pid_vz.get_kp()
            config.vz_ki = self.pid_vz.get_ki()
            config.vz_kd = self.pid_vz.get_kd()

            config.vx_kp = self.pid_vx.get_kp()
            config.vx_ki = self.pid_vx.get_ki()
            config.vx_kd = self.pid_vx.get_kd()

            config.vy_kp = self.pid_vy.get_kp()
            config.vy_ki = self.pid_vy.get_ki()
            config.vy_kd = self.pid_vy.get_kd()


            config.yaw_kp = self.pid_yaw.get_kp()
            config.yaw_ki = self.pid_yaw.get_ki()
            config.yaw_kd = self.pid_yaw.get_kd()

            config.yaw_r_kp = self.pid_yaw_rate.get_kp()
            config.yaw_r_ki = self.pid_yaw_rate.get_ki()
            config.yaw_r_kd = self.pid_yaw_rate.get_kd()

            self.config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers
            self.pid_z.set_kp(config.z_kp)
            self.pid_z.set_ki(config.z_ki)
            self.pid_z.set_kd(config.z_kd)

            self.pid_vz.set_kp(config.vz_kp)
            self.pid_vz.set_ki(config.vz_ki)
            self.pid_vz.set_kd(config.vz_kd)

            self.pid_vx.set_kp(config.vx_kp)
            self.pid_vx.set_ki(config.vx_ki)
            self.pid_vx.set_kd(config.vx_kd)

            self.pid_vy.set_kp(config.vy_kp)
            self.pid_vy.set_ki(config.vy_ki)
            self.pid_vy.set_kd(config.vy_kd)

            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_ki(config.yaw_ki)
            self.pid_yaw.set_kd(config.yaw_kd)

            self.pid_yaw_rate.set_kp(config.yaw_r_kp)
            self.pid_yaw_rate.set_ki(config.yaw_r_ki)
            self.pid_yaw_rate.set_kd(config.yaw_r_kd)

        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('mav_xyz_controller')
    height_ctl = HeightControl()
    height_ctl.run()

