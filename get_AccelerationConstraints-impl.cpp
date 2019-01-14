#pragma once
#include <cmath>
#define PI 3.141592653

template <size_t STATE_DIM, size_t CONTROL_DIM>
Eigen::Vector3d AccelerationLimits<STATE_DIM, CONTROL_DIM>::get_AccelerationLimits(const double & v)
{	
	size_t v_index = std::min(12001, int(round(100 * v)));

	Eigen::Matrix<double, 2, Eigen::Dynamic> a;
	
	Eigen::Vector3d Limits;

	Limits(0) = a_lateral_max_list_.col(v_index)(1);

	Limits(1) = a_forward_max_list_.col(v_index)(1);

	Limits(2) = a_backward_max_list_.col(v_index)(1);

	return Limits;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
AccelerationLimits<STATE_DIM, CONTROL_DIM>::AccelerationLimits() : delta_x_(0.01)
{
	xValue.resize(5);
	yValue.resize(5);
	xValue << 0.0, 10.0, 43.0, 52.0, 200.0;
	yValue << 1.0, 14.0, 28.0, 33.0, 33.0;
	interpl(xValue, yValue, delta_x_, a_lateral_max_list_);

	xValue.resize(8);
	yValue.resize(8);
	xValue << 0.0, 10.0, 20.0, 35.0, 52.0, 79.0, 83.0, 200.0;
	yValue << 2.0, 13.0, 18.0, 18.0, 15.0, 6.0, 1.0, 1.0;
	interpl(xValue, yValue, delta_x_, a_forward_max_list_);

	xValue.resize(6);
	yValue.resize(6);
	xValue << 0.0, 30.0, 40.0, 52.0, 76.0, 200.0;
	yValue << 11.0, 13.0, 24.0, 30.0, 40.0, 40.0;
	interpl(xValue, yValue, delta_x_, a_backward_max_list_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void AccelerationLimits<STATE_DIM, CONTROL_DIM>::push_back(Eigen::Matrix<double, 2, Eigen::Dynamic>& m, Eigen::Vector2d vectorAppend)
{
	m.conservativeResize(m.rows(), m.cols() + 1);
	m.rightCols(1) = vectorAppend;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
int AccelerationLimits<STATE_DIM, CONTROL_DIM>::interpl(
	const Eigen::VectorXd & xValue,
	const Eigen::VectorXd & yValue,
	const double& delta_x_,
	Eigen::Matrix<double, 2, Eigen::Dynamic>& InterpolationResult)
{
	size_t xSize = xValue.rows();
	size_t ySize = yValue.rows();

	if (xSize != ySize)
	{
		std::cout << "The columns of x and y are different!" << std::endl;
		return -1;
	}
	else
	{	
		Eigen::Vector2d vectorAppend;

		vectorAppend << xValue(0), yValue(0);
		push_back(InterpolationResult, vectorAppend);
		for (size_t i = 0; i < xSize - 1; ++i)
		{
			double delta_y_ = (yValue(i + 1) - yValue(i)) / (xValue(i + 1) - xValue(i)) * delta_x_;

			size_t n = (xValue(i+1) - xValue(i)) / delta_x_;

			for (size_t j = 0; j < n && (vectorAppend(0) <= 120);j++)
			{
				vectorAppend << vectorAppend(0) + delta_x_, vectorAppend(1) + delta_y_;
				push_back(InterpolationResult, vectorAppend);
			}
		}
		return 0;
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void AccelerationLimits<STATE_DIM, CONTROL_DIM>::get_AccelerationConstraints(std::vector< Eigen::Matrix<double, 1, 2> >& A, std::vector< double >& b, const ct::core::StateVector<STATE_DIM, double>& x_previous)
{	
	if ( (A.size() != 16) || (b.size() != 16) )
	{
		A.resize(16);
		b.resize(16);
	}

	double v;
	v = std::sqrt(std::pow(x_previous(2), 2) + std::pow(x_previous(3), 2));
	Eigen::Vector3d a_max( get_AccelerationLimits(v) );

	double delta_angle = 2 * PI / 16;
	for (size_t i = 1; i < 17; i++)
	{
		double c = std::cos(i * delta_angle);
		double s = std::sin(i * delta_angle);

		double ay_max( a_max(0) );
		double ax_max;

		if (c > 0)
			ax_max = a_max(1);
		else
			ax_max = a_max(2);

		Eigen::Matrix<double, 1, 2> k;
		k << ay_max * c, ax_max * s;

		Eigen::Matrix<double, 2, 2> l;
		l << x_previous(2), x_previous(3), -x_previous(3), x_previous(2);

		Eigen::Matrix<double, 1, 2> A_i = 1 / std::sqrt(std::pow(v, 2) + 0.01) * k * l;
		double b_i = ax_max * ay_max;

		A[i - 1] = A_i;
		b[i - 1] = b_i;
	}
}