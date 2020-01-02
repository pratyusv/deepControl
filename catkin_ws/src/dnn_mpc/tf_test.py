#!/usr/bin/python
import tensorflow as tf
import numpy as np
import argparse
import matplotlib.pyplot as plt

model = 'f450'
data_dir="./data/" + model + '/'
output_dir="./output_dir/rpyth_mpc/" + model + '/'
mean_std_dir = './data/' + model + '/mean_std/'

test_X = np.load(data_dir + 'test/test_X.npy')
test_U = np.load(data_dir + 'test/test_U.npy')

Xtr = test_X
Ytr = test_U

# Xtr = test_X[:500]
# Ytr = test_U[:500]

# Xtr = [[0.14604905, 0.51560138, 0.27780092, 0.8901413 , 0.90359776, 0.20449533, 0.27998261, 0.48336262, 0.36420253, 0.37711515]]
# Xtr = np.reshape(Xtr, (1,10))

# Ytr = np.random.rand(1,4)


##########################
# Normalize
##########################

mean = np.load(mean_std_dir + 'mean.npy')
std = np.load( mean_std_dir + 'std.npy')

print('mean :', mean.shape)
print('std :', std.shape)
print('Xtr: ', Xtr.shape)
print('Ytr: ', Ytr.shape)
# exit()

Xtr = (Xtr -mean)/std
# print(Xtr)
# print(Xtr.std(axis=0))
# exit()

#########################
# Network Structure
##########################

output_dim = 4
input_dim = 10

X = tf.placeholder(tf.float32, shape=[None,input_dim])
U = tf.placeholder(tf.float32, shape=[None,4])

with tf.variable_scope('rpyth'):
	x = tf.layers.dense(X, 64)
	# x = tf.contrib.layers.layer_norm(x, center=True, scale=True)
	x = tf.nn.relu(x)

	x = tf.layers.dense(x, 64)
	# x = tf.contrib.layers.layer_norm(x, center=True, scale=True)
	x = tf.nn.relu(x)

	x = tf.layers.dense(x, output_dim) #, kernel_initializer=tf.random_uniform_initializer(minval=-3e-3, maxval=3e-3))
	controls = tf.nn.tanh(x)
	predics = controls

	# controls = tf.multiply(x, action_bound)
u_cost = tf.losses.absolute_difference(labels=U, predictions=controls*1200.)


saver = tf.train.Saver()

# optimizer = tf.train.AdamOptimizer().minimize(cost)
init = tf.global_variables_initializer()

with tf.Session() as sess:
	saver.restore(sess, output_dir + "model.ckpt")
	print("Model Loaded Successfully")

	predictions, c = sess.run([controls,u_cost],feed_dict={X: Xtr ,U: Ytr})
	predictions = predictions
	# predictions[:,3] = np.add(np.multiply(predictions[:,3], 10.34), predictions[:,3])
	print("Cost = ", '{:.5f}'.format(c))

	# print(predictions)
	# exit()

	print("predictions shape = ",predictions.shape)
	print("Ytr         shape = ",Ytr.shape)
	

	plt.rcParams["figure.figsize"] = (15,10)
	fig,ax = plt.subplots(nrows=2,ncols=2)

	start  = 0
	stop = Ytr.shape[0]

	# ax[0].set_ylim(-1,1)
	ax[0][0].plot(Ytr[start:stop,0],'b',label='grnd_truth')
	ax[0][0].plot(predictions[start:stop,0],'r',label='predictions')
	ax[0][0].legend()
	ax[0][0].set_ylabel('roll')
	ax[0][0].set_xlabel('time')


	# ax[1].set_ylim(-0.6,0.6)
	ax[0][1].plot(Ytr[start:stop,1],'b',label='grnd_truth')
	ax[0][1].plot(predictions[start:stop,1],'r',label='predictions')
	ax[0][1].legend()
	ax[0][1].set_ylabel('pitch')
	ax[0][1].set_xlabel('time')


	# # ax[2].set_ylim(-0.6,0.6)
	ax[1][0].plot(Ytr[start:stop,2],'b',label='grnd_truth')
	ax[1][0].plot(predictions[start:stop,2],'r',label='predictions')
	ax[1][0].legend()
	ax[1][0].set_ylabel('yaw_rate')
	ax[1][0].set_xlabel('time')

	ax[1][1].plot(Ytr[start:stop,3],'b',label='grnd_truth')
	ax[1][1].plot(predictions[start:stop,3],'r',label='predictions')
	ax[1][1].legend()
	ax[1][1].set_ylabel('thrust')
	ax[1][1].set_xlabel('time')

	fname = 'plot.png'

	fig.savefig(fname, dpi=None, facecolor='w', edgecolor='w',
        orientation='portrait', papertype=None, format=None,
        transparent=False, bbox_inches=None, pad_inches=0.1,
        frameon=None)


	plt.show()
	
