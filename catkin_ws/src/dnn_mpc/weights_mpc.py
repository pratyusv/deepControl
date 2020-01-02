#!/usr/bin/env python

import math
import numpy as np
import rospy
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
import argparse


PI = 3.14

model = '/f450'

model_path = 'f450'

weights_dir = './output_dir/rpyth_mpc/'+ model_path +'/weights/'
mean_std_dir = './data/'+ model_path +'/mean_std/'



weights_1 = np.load(weights_dir + 'wt_layers_1.npy').transpose()
bias_1   = np.load(weights_dir + 'b_layers_1.npy').transpose()


weights_2 = np.load(weights_dir + 'wt_layers_2.npy').transpose()
bias_2  = np.load(weights_dir + 'b_layers_2.npy').transpose()


weights_3 = np.load(weights_dir + 'wt_layers_3.npy').transpose()
bias_3   = np.load(weights_dir + 'b_layers_3.npy').transpose()


mean = np.load(mean_std_dir + 'mean.npy')
std  = np.load(mean_std_dir + 'std.npy')

print('mean', mean)
print('std', std)
exit()




target_pose = PoseStamped()
def target_cb(data):
	global target_pose
	target_pose = data

mpc_u = RollPitchYawrateThrust()
def mpc_cb(data):
	global mpc_u
	mpc_u = data



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


def do_control():
	
	odom_sub = rospy.Subscriber(model+'/ground_truth/odometry',Odometry, odom_cb, queue_size=100)
	mpc_sub = rospy.Subscriber(model + '/rpyth', RollPitchYawrateThrust,mpc_cb, queue_size=100)	
	target_pose_sub = rospy.Subscriber(model + '/command/pose',PoseStamped,target_cb, queue_size=100)

	motor_speed_pub = rospy.Publisher(model+ "/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust,queue_size=100)
	

	rospy.init_node('controller',anonymous=True)
	rate =  rospy.Rate(50.0)

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

		rs_f = (float(odom.twist.twist.angular.x))
		ps_f = (float(odom.twist.twist.angular.y))
		ys_f = (float(odom.twist.twist.angular.z))


		
		target = np.array([target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z, 0.,0.,0., 0.,0., 0.,0.],ndmin=2)
		# target = np.array([1.,1.,1., 0.,0.,0., 0.,0., 0.,0.],ndmin=2)
		state = np.array([x_f,y_f,z_f, vx_f,vy_f,vz_f, r_f,p_f, rs_f,ps_f],ndmin=2)

		inputs = state - target

		# Xtr = [[0.14604905, 0.51560138, 0.27780092, 0.8901413 , 0.90359776, 0.20449533, 0.27998261, 0.48336262, 0.36420253, 0.37711515]]
		# Xtr = np.reshape(Xtr, (1,10))

		inputs = (inputs-mean)/std
		motor_controls = np.ravel(get_actions(inputs))

			


				
		actuator = RollPitchYawrateThrust()
		actuator.header.stamp = rospy.Time.now()
		actuator.roll = motor_controls[0]
		actuator.pitch = motor_controls[1]
		# actuator.yaw_rate = motor_controls[2]
		actuator.thrust.z = (motor_controls[3]*10.34)+(10.34)

		# actuator.roll = mpc_u.roll
		# actuator.pitch = mpc_u.pitch
		actuator.yaw_rate = mpc_u.yaw_rate
		# actuator.thrust.z = mpc_u.thrust.z

		print('roll: {:.5f}, pitch: {}, yaw_rate: {}, thrust: {}'.format(actuator.roll, actuator.pitch, actuator.yaw_rate, actuator.thrust.z))



		motor_speed_pub.publish(actuator)
		
		rate.sleep()


def get_controlCost(state, reference):
	a = 5
	return a



def get_actions(inputs):

	x = np.matmul(inputs, weights_1.transpose())
	x = np.add(x, bias_1)
	x = np.maximum(x, 0, x)


	x = np.matmul(x, weights_2.transpose())
	x = np.add(x, bias_2)
	x = np.maximum(x, 0, x)


	x = np.matmul(x, weights_3.transpose())
	x = np.add(x, bias_3)
	controls = np.tanh(x)
	# print(controls)
	# exit()
		

	return controls
	


if __name__ == '__main__':
	try:
		do_control()

	except rospy.ROSInterruptException:
		pass