/*
	Get the acceleration constraints for every state. approximating with n = 16 tangent constraints

*/
#pragma once 
#include <iostream>
#include <cmath>
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "AccelerationLimits.h"

#define PI 3.141592653


void get_AccelerationConstraints(Eigen::Matrix<double, 1, 2> & A, double &b, const ct::core::StateVector<4, double> & x, const int & i)
{	
	static std::shared_ptr<AccelerationLimits> AccelerationLimits_(
		new AccelerationLimits);
	double v;
	v = std::sqrt(std::pow(x(2), 2) + std::pow(x(3), 2));



	double delta_angle = 2 * PI / 16;


		double c = std::cos(i * delta_angle);
		double s = std::sin(i * delta_angle);
		Eigen::Vector3d a_max;
		
		AccelerationLimits_->get_AccelerationLimits(a_max, v);

		double ax_max;
		double ay_max(a_max(0));
		if (c > 0)
			ax_max = a_max(1);
		else
			ax_max = a_max(2);
		
		Eigen::Matrix<double, 1, 2> k;
		k << ay_max * c, ax_max * s;
		Eigen::Matrix<double, 2, 2> l;
		l << x(2), x(3), -x(3), x(2);
		A = 1 / std::sqrt(std::pow(v, 2) + 0.01) * k * l;
   		b = ax_max * ay_max;

}