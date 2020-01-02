/*****************************************************************************
-----------PRATYUSH VARSHNEY-----------------
------------pratyushvarshney@cse.iitk.ac.in-------------
*****************************************************************************/

#include <iostream>
#include <params.hpp>

using namespace Eigen;

#define NO_OF_STATES 10


float max(float a, float b){
  if(a < b)
    return b;
  else
    return a;
}

struct controls{
    double roll_sp, pitch_sp, thrust_sp;
};


typedef struct controls controls;

controls generateControls(double x, double y, double z, double vx, double vy, double vz, double roll, double pitch, double rs, double ps, double target_x, double target_y, double target_z)
{

  /*Control structure*/
  controls u;
  weights np;

  VectorXf target(NO_OF_STATES);
  /* Position */
  target(0) = target_x;
  target(1) = target_y;
  target(2) = target_z;

  /* Velocity */
  target(3) = 0.0;
  target(4) = 0.0;
  target(5) = 0.0;

  /* Euler Angles */
  target(6) = 0.0;
  target(7) = 0.0;

  /* Angular Velocity*/
  target(8) = 0.0;
  target(9) = 0.0;

  VectorXf state_(NO_OF_STATES);
  /* Position */
  state_(0) = x;
  state_(1) = y;
  state_(2) = z;

  /* Velocity */
  state_(3) = vx;
  state_(4) = vy;
  state_(5) = vz;

  /* Euler Angles */
  state_(6) = roll;
  state_(7) = pitch;

  /* Angular Velocity*/
  state_(8) = rs;
  state_(9) = ps;

  Matrix<float, 1, NO_OF_STATES> input_;

  for (int i = 0; i < NO_OF_STATES; i++)
    input_(0, i) = state_(i) - target(i);

  /* Normalization of Input */
  for (int i = 0; i < NO_OF_STATES; i++)
    input_(0, i) = (input_(0, i) - np.mean(i, 0)) / np.std(i, 0);
  
  VectorXf controls_(3);

  /* LAYER 1 */
  Matrix<float, 1, 64> op1_w = (input_ * np.layer_1_weight.transpose());
  Matrix<float, 1, 64> op1 = op1_w + np.bias_1.transpose();

  /* ReLu for Layer 1*/
  for (int i = 0; i < 64; i++)
    op1(0, i) = max(0, op1(0, i));

  /* LAYER 2 */
  Matrix<float, 1, 64> op2_w = (op1 * np.layer_2_weight.transpose());
  Matrix<float, 1, 64> op2 = op2_w + np.bias_2.transpose();

  /* ReLu for Layer 2*/
  for (int i = 0; i < 64; i++)
    op2(0, i) = max(0, op2(0, i));

  /* LAYER 3 */
  Matrix<float, 1, 2> op3_w = (op2 * np.layer_3_weight.transpose());
  Matrix<float, 1, 2> op3_b = op3_w + np.bias_3.transpose();
  ArrayWrapper<Matrix<float, 1, 2>> op3(op3_b);

  /* tanh activation function */
  op3.tanh();

  /* return control structure */
  u.roll_sp = op3(0, 0);
  u.pitch_sp = op3(0, 1);
  u.thrust_sp = op3(0, 2);

  return u;
}
