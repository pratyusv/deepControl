#ifndef INCLUDE_PARAMS_HPP_
#define INCLUDE_PARAMS_HPP_

#include <Eigen/Dense>
#include <iostream>
#include "mc_weights.h"
#include "mc_bias.h"
#include "mc_mean.h"
#include "mc_std.h"
using namespace Eigen;


class weights
{
public:
	Eigen::Matrix<float,WEIGHTS_1_ROWS, WEIGHTS_1_COLS> layer_1_weight;
	Eigen::Matrix<float,WEIGHTS_2_ROWS, WEIGHTS_2_COLS> layer_2_weight;
	Eigen::Matrix<float,WEIGHTS_3_ROWS, WEIGHTS_3_COLS> layer_3_weight;

	Eigen::Matrix<float,BIAS_1_ROWS, 1> bias_1;
	Eigen::Matrix<float,BIAS_2_ROWS, 1> bias_2;
	Eigen::Matrix<float,BIAS_3_ROWS, 1> bias_3;

	Eigen::Matrix<float,MEAN_ROWS,1> mean;
	Eigen::Matrix<float,STD_ROWS,1> std;
	// Eigen::VectorXf mean(6);
	// Eigen::VectorXf std(6);

	weights (){
		layer_1_weight << WEIGHTS_1;
		layer_2_weight << WEIGHTS_2;
		layer_3_weight << WEIGHTS_3;

		bias_1 << BIAS_1;
		bias_2 << BIAS_2;
		bias_3 << BIAS_3;

		mean << MEAN;
		std << STD;
	}
};


#endif 