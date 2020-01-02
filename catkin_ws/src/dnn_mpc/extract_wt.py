import tensorflow as tf
import numpy as np

model = 'f450'
output_dir="./output_dir/rpyth_mpc/" + model + '/'

output_dim = 4
input_dim = 10


X = tf.placeholder(tf.float32, shape=[None, input_dim])
U = tf.placeholder(tf.float32, shape=[None, output_dim])


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



with tf.Session() as sess:
	saver = tf.train.Saver()
	saver.restore(sess, output_dir + 'model.ckpt')

	var_list = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='rpyth')

	wt_layer_1 = sess.run(var_list[0])
	b_layer_1 = sess.run(var_list[1])

	wt_layer_2 = sess.run(var_list[2])
	b_layer_2 = sess.run(var_list[3])

	wt_layer_3 = sess.run(var_list[4])
	b_layer_3 = sess.run(var_list[5])


	print("layer 1  wt: {} | bias: {}".format(wt_layer_1.shape, b_layer_1.shape))
	print("layer 2  wt: {} | bias: {}".format(wt_layer_2.shape, b_layer_2.shape))
	print("layer 3  wt: {} | bias: {}".format(wt_layer_3.shape, b_layer_3.shape))

	

	np.save(output_dir + 'weights/wt_layers_1.npy', wt_layer_1)
	np.save(output_dir + 'weights/wt_layers_2.npy', wt_layer_2)
	np.save(output_dir + 'weights/wt_layers_3.npy', wt_layer_3)

	np.save(output_dir + 'weights/b_layers_1.npy', b_layer_1)
	np.save(output_dir + 'weights/b_layers_2.npy', b_layer_2)
	np.save(output_dir + 'weights/b_layers_3.npy', b_layer_3)