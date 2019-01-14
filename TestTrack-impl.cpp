#pragma once

#include <cmath>
#define PI 3.141592653

template <size_t STATE_DIM, size_t CONTROL_DIM>
TestTrack<STATE_DIM, CONTROL_DIM>::TestTrack() : trackWidth(7.)
{	
	center.resize(1);
	left.resize(1);
	right.resize(1);
	forward_vector.resize(1);
	yaw.resize(1);

	center[0] << 0., 0.;
	left[0] << 0., trackWidth / 2.;
	right[0] << 0., - trackWidth / 2.;
	forward_vector[0] << 1., 0.;
	yaw[0] = 0.;

	add_turn(0, 76);
    add_turn(-0.25, 50);
    add_turn(-0.25, 8);
    add_turn(-0.1, 30);
    add_turn(0.1, 30);
    add_turn(0.5, 15);
    add_turn(-0.5, 30);
    add_turn(0.5, 15);
    add_turn(0, 60);
    add_turn(-0.25, 10);
    add_turn(-0.25, 20);
    add_turn(0, 55);
    add_turn(-0.25, 90.3);
    add_turn(0, 5.3);
    add_turn(-0.25, 20);

    center.erase( center.begin() );
    left.erase( left.begin() );
    right.erase( right.begin() );
    forward_vector.erase( forward_vector.begin() );
    yaw.erase( yaw.begin() );

    left_eigen.resize(left.size(), 2);
    right_eigen.resize(right.size(), 2);

    for (size_t i = 0; i < left.size(); i++)
    {
    	left_eigen(i, 0) = (left[i])(0);
    	left_eigen(i, 1) = (left[i])(1);
    	right_eigen(i, 0) = (right[i])(0);
    	right_eigen(i, 1) = (right[i])(1);
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void TestTrack<STATE_DIM, CONTROL_DIM>::add_turn(const double& phi, const double& L)
{
	double kappa = (phi * 2. * PI) / L;

	size_t N(40);

	double ds = L / N;

	for (size_t i = 1; i <= N; i++)
	{
		yaw.push_back(yaw.back() + kappa * ds);

		double c = std::cos( yaw.back() );
		double s = std::sin( yaw.back() );

		Eigen::Matrix<double, 2, 1> f;
		f << c, s;
		Eigen::Matrix<double, 2, 1> n;
		n << -s, c;

		center.push_back(center.back() + ds * f);
		left.push_back(center.back() + n * trackWidth / 2. );
		right.push_back(center.back() - n * trackWidth / 2.);

		forward_vector.push_back(f);
	}
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void TestTrack<STATE_DIM, CONTROL_DIM>::get_Track(Eigen::Matrix<double, 2, 1>& left_,
	Eigen::Matrix<double, 2, 1>& right_,
	Eigen::Matrix<double, 2, 1>& forward_vector_,
	const ct::core::StateVector<STATE_DIM, double>& previous_state)
{
	size_t idx = get_CheckPointIndex(previous_state);

	right_ = right[idx];
	left_ = left[idx];
	forward_vector_ = forward_vector[idx];
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void TestTrack<STATE_DIM, CONTROL_DIM>::get_ForwardVector(Eigen::Matrix<double, 2, 1>& forward_vector_,
	const ct::core::StateVector<STATE_DIM, double>& previous_state)
{
	size_t idx = get_CheckPointIndex(previous_state);

	forward_vector_ = forward_vector[idx];
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
size_t TestTrack<STATE_DIM, CONTROL_DIM>::get_CheckPointIndex(const ct::core::StateVector<STATE_DIM, double>& previous_state)
{
	size_t idx(0);
	double diff(std::numeric_limits<double>::infinity());
	double diff_temp;

	for (size_t i = 0; i < center.size(); i++)
	{
		diff_temp = pow((center[i])(0) - previous_state(0), 2) + pow((center[i])(1) - previous_state(1), 2);
		if (diff_temp < diff)
		{
			diff = diff_temp;
			idx = i;
		}
	}

	return idx;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
void TestTrack<STATE_DIM, CONTROL_DIM>::plot_Track(ct::core::StateVectorArray<STATE_DIM, double> xSolution_)
{
	using namespace ct::core;

	plot::ion();
	plot::figure();


	plot::plot(left_eigen.col(0), left_eigen.col(1));
	plot::plot(right_eigen.col(0), right_eigen.col(1));

	std::vector<double> x;
	std::vector<double> y;
	for (auto state : xSolution_)
	{
		x.push_back(state(0));
		y.push_back(state(1));
	}

	plot::plot(x, y);

	plot::show();
}