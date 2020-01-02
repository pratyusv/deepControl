import numpy as np
import pandas as pd

model = 'f450'
data_dir="./data/" + model + '/'
mean_std_dir = './data/' + model + '/mean_std/'
weights_dir = './output_dir/rpyth_mpc/'+ model +'/weights/'


inputs_data 	= np.load(data_dir + model + '_input_state.npy')
setpoints_data  = np.load(data_dir + model + '_setpoints.npy')
controls_data   = np.load(data_dir + model + '_controls.npy')

weights_1 = np.load(weights_dir + 'wt_layers_1.npy').transpose()
bias_1   = np.load(weights_dir + 'b_layers_1.npy').transpose()


weights_2 = np.load(weights_dir + 'wt_layers_2.npy').transpose()
bias_2  = np.load(weights_dir + 'b_layers_2.npy').transpose()


weights_3 = np.load(weights_dir + 'wt_layers_3.npy').transpose()
bias_3   = np.load(weights_dir + 'b_layers_3.npy').transpose()


mean = np.load(mean_std_dir + 'mean.npy')
std  = np.load(mean_std_dir + 'std.npy')


print("layer 1  wt: {} | bias: {}".format(weights_1.shape, bias_1.shape))
print("layer 2  wt: {} | bias: {}".format(weights_2.shape, bias_2.shape))
print("layer 3  wt: {} | bias: {}".format(weights_3.shape, bias_3.shape))

print("mean : {}".format(mean.shape))
print("std:   {}".format(std.shape))



print("Input     Data :", inputs_data.shape)
print("Setpoints Data :", setpoints_data.shape)
print("Controls  Data :", controls_data.shape)


df_inputs = pd.DataFrame(inputs_data)
df_setpoints = pd.DataFrame(setpoints_data)
df_controls = pd.DataFrame(controls_data)

print("----Inputs----")
print(df_inputs.describe())

print("----Setpoints----")
print(df_setpoints.describe())

print("----Controls----")
print(df_controls.describe())