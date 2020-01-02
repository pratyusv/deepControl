#!/usr/bin/env python


import math
import numpy as np
import rospy
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import MultiDOFJointTrajectory
import argparse

from std_msgs.msg import String

PI = 3.14

weights_dir = './wt/'
output_dir = './wt/'

weights_1 = np.load(weights_dir + 'weights_1.npy').transpose()
bias_1   = np.load(weights_dir + 'bias_1.npy').transpose()


weights_2 = np.load(weights_dir + 'weights_2.npy').transpose()
bias_2  = np.load(weights_dir + 'bias_2.npy').transpose()


weights_3 = np.load(weights_dir + 'weights_3.npy').transpose()
bias_3   = np.load(weights_dir + 'bias_3.npy').transpose()

mean = np.load(output_dir + 'mean.npy')
std  = np.load(output_dir + 'std.npy')

mean_x = np.load(output_dir_x + 'mean.npy')
std_x  = np.load(output_dir_x + 'std.npy')





# print('mean', mean)
# print('std', std)
# exit()

target_flag_pub  = False

vrpn = TwistStamped()
def vrpn_cb(data):
	global vrpn
	vrpn = data


target_pose = MultiDOFJointTrajectory()
def target_cb(data):
	global target_pose
	target_pose = data
	global target_flag_pub
	target_flag_pub = True

mpc_u = RollPitchYawrateThrust()
def mpc_cb(data):
	global mpc_u
	mpc_u = data


state_machine = String()
def state_cb(data):
	global state_machine
	state_machine = data


def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z


odom = Odometry()
def odom_cb(data):
	global odom
	odom = data


def do_control( ):
	
	odom_sub = rospy.Subscriber('/f_450/odometry',Odometry, odom_cb, queue_size=100)
	vrpn_sub = rospy.Subscriber('/f_450/vrpn_client_node/f_450/twist',TwistStamped,vrpn_cb, queue_size=100)
	state_sub = rospy.Subscriber('/f_450/state_machine/state_info', String, state_cb, queue_size=100)
	target_pose_sub = rospy.Subscriber('/f_450/command/current_reference', MultiDOFJointTrajectory ,target_cb, queue_size=100)
	mpc_sub = rospy.Subscriber('/f_450/command/roll_pitch_yawrate_thrust',RollPitchYawrateThrust, mpc_cb, queue_size=100)

	control_pub = rospy.Publisher("/f_450/mavros/setpoint_raw/roll_pitch_yawrate_thrust", RollPitchYawrateThrust,queue_size=100)

	rospy.init_node('controller',anonymous=True)
	rate =  rospy.Rate(50.0)
	global target_flag_pub	

	# intial Target
	target = np.array([0.0,0.0,0.5, 0.,0.,0., 0.,0.],ndmin=2)
	

	while not rospy.is_shutdown():

		x_f = odom.pose.pose.position.x
		y_f = odom.pose.pose.position.y
		z_f = odom.pose.pose.position.z

		vx_f = odom.twist.twist.linear.x
		vy_f = odom.twist.twist.linear.y
		vz_f = odom.twist.twist.linear.z

		(roll,pitch, yaw) = quaternion_to_euler_angle(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x , odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
		r_f = math.radians(roll)
		p_f = math.radians(pitch)
		yaw_f = math.radians(yaw)

		rs_f = float(vrpn.twist.angular.x)
		ps_f = float(vrpn.twist.angular.y)
		ys_f = float(vrpn.twist.angular.z)

		if target_flag_pub == True:
			target = np.array([target_pose.points[0].transforms[0].translation.x , target_pose.points[0].transforms[0].translation.y,target_pose.points[0].transforms[0].translation.z, 0.,0.,0., 0.,0.],ndmin=2)
		
		

		target_flag_pub = False


		state = np.array([x_f,y_f,z_f, vx_f,vy_f,vz_f, r_f,p_f],ndmin=2)

		inputs = state - target
		inputs_x = (inputs-mean_x)/std_x
		inputs = (inputs-mean)/std

		print(str(state_machine.data))

		if str(state_machine.data) == 'PositionHold':
			
			print('MPC: ')
			controls = np.ravel(get_actions(inputs))
			controls_x = np.ravel(get_actions_x(inputs_x))
			rpyth_mpc = RollPitchYawrateThrust()

			rpyth_mpc.header.stamp = rospy.Time.now()	
			rpyth_mpc.roll = controls[0]
			rpyth_mpc.pitch = controls_x[1]
			rpyth_mpc.yaw_rate = mpc_u.yaw_rate
			rpyth_mpc.thrust.z = (controls[2] * 7.5) + 7.5	
			control_pub.publish(rpyth_mpc)
			

		elif str(state_machine.data) == 'RcTeleOp' or str(state_machine.data) == 'RemoteControl' or str(state_machine.data) == 'HaveOdometry' :
			print('MPC')
			rpyth_mpc = RollPitchYawrateThrust()

			rpyth_mpc.header.stamp = rospy.Time.now()	
			rpyth_mpc.roll = mpc_u.roll
			rpyth_mpc.pitch = mpc_u.pitch
			rpyth_mpc.yaw_rate = mpc_u.yaw_rate
			rpyth_mpc.thrust.z = mpc_u.thrust.z
			
			
			control_pub.publish(rpyth_mpc)


		rate.sleep()



def get_actions(inputs):

	x = np.matmul(inputs, weights_1)
	x = np.add(x, bias_1)
	x = np.maximum(x, 0, x)


	x = np.matmul(x, weights_2)
	x = np.add(x, bias_2)
	x = np.maximum(x, 0, x)

	x = np.matmul(x, weights_3)
	x = np.add(x, bias_3)
	controls = np.tanh(x)	
		

	return controls
	


if __name__ == '__main__':
	try:
		do_control()

	except rospy.ROSInterruptException:
		pass
