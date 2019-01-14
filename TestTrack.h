#pragma once
#include <vector>
#include <Eigen/Dense>
#include <ct/core/core.h>

template <size_t STATE_DIM, size_t CONTROL_DIM>
class TestTrack
{
public:
	TestTrack();

	~TestTrack() = default;

	void get_Track(Eigen::Matrix<double, 2, 1>& left_,
		Eigen::Matrix<double, 2, 1>& right_,
		Eigen::Matrix<double, 2, 1>& forward_vector_,
		const ct::core::StateVector<STATE_DIM, double>& previous_state);

	void get_ForwardVector(Eigen::Matrix<double, 2, 1>& forward_vector_,
		const ct::core::StateVector<STATE_DIM, double>& previous_state);

	void plot_Track(ct::core::StateVectorArray<STATE_DIM, double> xSolution_);

protected:
	size_t get_CheckPointIndex(const ct::core::StateVector<STATE_DIM, double>& previous_state);

	void add_turn(const double& phi, const double& L);

private:
	std::vector< Eigen::Matrix<double, 2, 1> > left;
	std::vector< Eigen::Matrix<double, 2, 1> > right;
	std::vector< Eigen::Matrix<double, 2, 1> > center;
	std::vector< double > yaw;
	std::vector< Eigen::Matrix<double, 2, 1> > forward_vector;

	Eigen::Matrix<double, Eigen::Dynamic, 2> left_eigen;
	Eigen::Matrix<double, Eigen::Dynamic, 2> right_eigen;

	const double trackWidth;
};

#include "TestTrack-impl.cpp"