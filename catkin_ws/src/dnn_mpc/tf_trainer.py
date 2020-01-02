import tensorflow as tf
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd

model = 'f450'
data_dir="./data/" + model + '/'
output_dir="./output_dir/rpyth_mpc/" + model + '/'
mean_std_dir = './data/' + model + '/mean_std/'


inputs_data 	= np.load(data_dir + model + '_input_state.npy')
setpoints_data  = np.load(data_dir + model + '_setpoints.npy')
controls_data   = np.load(data_dir + model + '_controls.npy')

# inputs_data 	= inputs_data[0:100000,:]
# setpoints_data 	= setpoints_data[0:100000,:]
# controls_data 	= controls_data[0:100000,:]

new_pos  = inputs_data[:,0:3] - setpoints_data[:,0:3]
data_X = np.concatenate((new_pos, inputs_data[:,3:6], inputs_data[:,6:8], inputs_data[:,9:11]), axis=1)
controls_data[:,3:4]= np.divide(np.subtract(np.multiply(controls_data[:,3:4],1.),10.34),10.34)
data_U = np.concatenate((controls_data[:,0:3], controls_data[:,3:4]), axis=1)


print('train_X :', data_X.shape)
print('train_Y :', data_U.shape)

# exit()


order = np.argsort(np.random.random(data_X.shape[0]))
data_X = data_X[order]
data_U = data_U[order]

new_range = (0.9*data_X.shape[0])
new_range = int(math.ceil(new_range))

train_X = data_X[0:new_range, :]
train_U = data_U[0:new_range, :]


test_X = data_X[new_range:,:]
test_U = data_U[new_range:,:]

print('train_X :',train_X.shape,'train_Y :',train_U.shape)
print('test_X :',test_X.shape,'test_Y :',test_U.shape)


df = pd.DataFrame(data_U)
print(df.describe())
# exit()

np.save(data_dir + 'train/train_X.npy', train_X)
np.save(data_dir + 'train/train_U.npy', train_U)

np.save(data_dir + 'test/test_X.npy', test_X)
np.save(data_dir + 'test/test_U.npy', test_U)




########################
# Normalize and shuffle
########################


mean  = train_X.mean(axis=0)
std   = train_X.std(axis=0)

train_X = (train_X - mean)/std
test_X = (test_X - mean)/std


np.save(mean_std_dir + 'mean.npy', mean)
np.save(mean_std_dir + 'std.npy', std)

print(train_X.mean(axis=0))
print(train_X.std(axis=0))
print('mean :',len(mean))
print('std :', len(std))
# exit()



####################
# Network Structure
####################

output_dim = 4
input_dim = 10

X = tf.placeholder(tf.float32, shape=[None, input_dim])
U = tf.placeholder(tf.float32, shape=[None, output_dim])
# action_bound = np.array([0.5, 0.5, 0.5, 8.33], ndmin=2)
NUM_EPOCHS = 500
BATCH_SIZE = 128

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


cost_to_minimize = tf.losses.huber_loss(labels=U, predictions=predics)

optimizer = tf.train.AdamOptimizer(learning_rate=1e-3).minimize(cost_to_minimize)
u_cost = tf.losses.absolute_difference(labels=U, predictions=predics)

num_batches = int(math.ceil(float(train_X.shape[0])/BATCH_SIZE))
test_num_batches = int(math.ceil(float(test_X.shape[0])/BATCH_SIZE))
print('train_num_batches', num_batches)
print('test_num_batches', test_num_batches)


plot_test = []
plot_u = []
with tf.Session() as sess:

	sess.run(tf.global_variables_initializer())
	saver = tf.train.Saver()

	for epoch in range(NUM_EPOCHS):
	## Training
		running_cost = 0.0
		u_running_cost = 0.0
		steps = range(num_batches)
		for step in steps:
				left = step*BATCH_SIZE
				right = min(left + BATCH_SIZE, train_X.shape[0])
				tx = train_X[left:right,:]
				tu = train_U[left:right,:]
				_, r_c, r_u = sess.run([optimizer ,cost_to_minimize, u_cost], feed_dict={X: tx, U:tu})
				running_cost += r_c
				u_running_cost += r_u

		epoch_cost   = running_cost/num_batches
		u_epoch_cost = u_running_cost/num_batches

		trajectory_cost = sess.run(u_cost, feed_dict={X:test_X, U: test_U})
		plot_test.append(u_epoch_cost)
		plot_u.append(u_epoch_cost)


		print(" Epoch:", '%04d' % (epoch+1), "train_cost=", "{:.9f}".format(epoch_cost) , "u_cost = {:.9f}".format(u_epoch_cost) ," test_cost=", "{:.9f}".format(trajectory_cost)) #, " learning_rate {:.8f}".format(_learning))

	save_path = saver.save(sess, output_dir+"model.ckpt")
	print("Model saved in file: %s" % save_path)

	plt.plot(plot_test)
	plt.plot(plot_u)
	plt.show()
