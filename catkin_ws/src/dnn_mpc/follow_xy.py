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

mass = 1.20
model = 'f450'
controller_type = 'dnn'
constant_thrust = mass * (10.34/1.45)

weights_dir = './output_dir/rpyth_mpc/'+ model +'/weights/'
data_dir = './data/'
mean_std_dir = './data/'+ model +'/mean_std/'


weights_1 = np.load(weights_dir + 'wt_layers_1.npy').transpose()
bias_1   = np.load(weights_dir + 'b_layers_1.npy').transpose()


weights_2 = np.load(weights_dir + 'wt_layers_2.npy').transpose()
bias_2  = np.load(weights_dir + 'b_layers_2.npy').transpose()


weights_3 = np.load(weights_dir + 'wt_layers_3.npy').transpose()
bias_3   = np.load(weights_dir + 'b_layers_3.npy').transpose()


mean = np.load(mean_std_dir + 'mean.npy')
std  = np.load(mean_std_dir + 'std.npy')


# weights_dir_x = './wt/bk2/'
# output_dir_x = './wt/bk2/'

# weights_1_x = np.load(weights_dir_x + 'weights_1.npy').transpose()
# bias_1_x   = np.load(weights_dir_x + 'bias_1.npy').transpose()


# weights_2_x = np.load(weights_dir_x + 'weights_2.npy').transpose()
# bias_2_x  = np.load(weights_dir_x + 'bias_2.npy').transpose()


# weights_3_x = np.load(weights_dir_x + 'weights_3.npy').transpose()
# bias_3_x   = np.load(weights_dir_x + 'bias_3.npy').transpose()


# weights_4 = np.load(weights_dir + 'weights_4.npy').transpose()
# bias_4   = np.load(weights_dir + 'bias_4.npy').transpose()


# mean_x = np.load(output_dir_x + 'mean.npy')
# std_x  = np.load(output_dir_x + 'std.npy')

# print('mean', mean)
# print('std', std)
# exit()

target_flag_pub  = False


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
	
	odom_sub = rospy.Subscriber('/'+model+'/ground_truth/odometry', Odometry, odom_cb, queue_size=100)
	target_pose_sub = rospy.Subscriber('/'+ model + '/command/current_reference', MultiDOFJointTrajectory ,target_cb, queue_size=100)
	mpc_sub = rospy.Subscriber('/' + model + '/rpyth', RollPitchYawrateThrust,mpc_cb, queue_size=100)

	control_pub = rospy.Publisher(model + "/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust,queue_size=100)

	rospy.init_node('controller',anonymous=True)
	rate =  rospy.Rate(50.0)
	global target_flag_pub	
	target = np.array([-1.6, -0.3,0.5, 0.,0.,0., 0.,0.],ndmin=2)
	

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

		# target = np.array([target_pose.pose.position.x ,target_pose.pose.position.y, target_pose.pose.position.z, 0.,0.,0., 0.,0.,0., 0.,0.,0.],ndmin=2)
		# state = np.array([x_f,y_f,z_f, vx_f,vy_f,vz_f, r_f,p_f,yaw_f, rs_f,ps_f,ys_f],ndmin=2)

		if target_flag_pub == True:
			target = np.array([target_pose.points[0].transforms[0].translation.x , target_pose.points[0].transforms[0].translation.y,target_pose.points[0].transforms[0].translation.z, 0.,0.,0., 0.,0., 0.,0.],ndmin=2)
		
		
		# print(target_flag_pub)
		target_flag_pub = False
		
		


		state = np.array([x_f,y_f,z_f, vx_f,vy_f,vz_f, r_f,p_f, rs_f,ps_f],ndmin=2)

		inputs = state - target
		inputs = (inputs-mean)/std

		controls = np.ravel(get_actions(inputs))
		rpyth_mpc = RollPitchYawrateThrust()

		rpyth_mpc.header.stamp = rospy.Time.now()

		if(controller_type == 'dnn'):
			
			rpyth_mpc.roll = controls[0]
			rpyth_mpc.pitch = controls[1]
			rpyth_mpc.yaw_rate = mpc_u.yaw_rate
			rpyth_mpc.thrust.z = (controls[3] * constant_thrust) + constant_thrust
			print('DNN: roll: {:.3f} pitch: {:.3f} thrust: {:3f}'.format(rpyth_mpc.roll, rpyth_mpc.pitch, rpyth_mpc.thrust.z))
			
		
		else:
			rpyth_mpc.roll = mpc_u.roll
			rpyth_mpc.pitch = mpc_u.pitch
			rpyth_mpc.yaw_rate = mpc_u.yaw_rate
			rpyth_mpc.thrust.z = mpc_u.thrust.z
			print('MPC: roll: {:.3f} pitch: {:.3f} thrust: {:3f}'.format(rpyth_mpc.roll, rpyth_mpc.pitch, rpyth_mpc.thrust.z))

		print('Thrust Error: {:.3f}'.format(abs(z_f - target_pose.points[0].transforms[0].translation.z)))
		control_pub.publish(rpyth_mpc)



		rate.sleep()




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
