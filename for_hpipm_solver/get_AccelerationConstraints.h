#pragma once
#include <ct/core/core.h>
#include <Eigen/Dense>
#include <cmath>

template <size_t STATE_DIM, size_t CONTROL_DIM>
class AccelerationLimits
{
public:
	AccelerationLimits();

	~AccelerationLimits() = default;

	void get_AccelerationConstraints(std::vector< Eigen::Matrix<double, 1, 2> >& A,
		std::vector< double >& b, 
		const ct::core::StateVector<STATE_DIM, double>& x_previous);

private:
	Eigen::Matrix<double, 2, Eigen::Dynamic> a_lateral_max_list_;
	Eigen::Matrix<double, 2, Eigen::Dynamic> a_forward_max_list_;
	Eigen::Matrix<double, 2, Eigen::Dynamic> a_backward_max_list_;

	Eigen::VectorXd xValue;
	Eigen::VectorXd yValue;

	double delta_x_;
	double delta_y_;

	void push_back(Eigen::Matrix<double, 2, Eigen::Dynamic>& m, Eigen::Vector2d vectorAppend);

	int interpl(
	const Eigen::VectorXd & xValue,
	const Eigen::VectorXd & yValue,
	const double& delta_x_,
	Eigen::Matrix<double, 2, Eigen::Dynamic>& InterpolationResult);

	Eigen::Vector3d get_AccelerationLimits(const double & v);
};

#include "get_AccelerationConstraints-impl.cpp"