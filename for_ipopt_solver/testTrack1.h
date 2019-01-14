#pragma once

#include <Eigen/Dense>
#include <cmath>

#define PI 3.141592653

struct CheckPoints
{
	Eigen::Matrix<double, 2, Eigen::Dynamic> left;
	Eigen::Matrix<double, 2, Eigen::Dynamic> right;
	Eigen::Matrix<double, 2, Eigen::Dynamic> center;
	Eigen::Matrix<double, 1, Eigen::Dynamic> yaw;
	Eigen::Matrix<double, 2, Eigen::Dynamic> forward_vector;
};

int add_turn(CheckPoints& checkpoints, const double& phi, const double&L, const double& width);
CheckPoints testTrack1();

/* 
int main(int argc, char const *argv[])
{
	CheckPoints checkpoints_;
	checkpoints_ = testTrack1();
	return 0;
}
*/
CheckPoints testTrack1()
{
	const double trackWidth = 7;

	CheckPoints checkpoints;
	checkpoints.left.resize(2, 1);
	checkpoints.left << 0, trackWidth / 2;
	checkpoints.right.resize(2, 1);
	checkpoints.right << 0, - trackWidth / 2;
	checkpoints.center.resize(2, 1);
	checkpoints.center << 0, 0;
	checkpoints.yaw.resize(1, 1);
	checkpoints.yaw << 0;
	checkpoints.forward_vector.resize(2, 1);
	checkpoints.forward_vector << 1, 0;
	
	add_turn(checkpoints, 0, 76, trackWidth);
    add_turn(checkpoints, -0.25, 50, trackWidth);
    add_turn(checkpoints, -0.25, 8, trackWidth);
    add_turn(checkpoints, -0.1, 30, trackWidth);
    add_turn(checkpoints, 0.1, 30, trackWidth);
    add_turn(checkpoints, 0.5, 15, trackWidth);
    add_turn(checkpoints, -0.5, 30, trackWidth);
    add_turn(checkpoints, 0.5, 15, trackWidth);
    add_turn(checkpoints, 0, 60, trackWidth);
    add_turn(checkpoints, -0.25, 10, trackWidth);
    add_turn(checkpoints, -0.25, 20, trackWidth);
    add_turn(checkpoints, 0, 55, trackWidth);
    add_turn(checkpoints, -0.25, 90.3, trackWidth);
    add_turn(checkpoints, 0, 5.3, trackWidth);
    add_turn(checkpoints, -0.25, 20, trackWidth);

    checkpoints.left.leftCols(checkpoints.left.cols() - 1) = checkpoints.left.rightCols(checkpoints.left.cols() - 1);
    checkpoints.left.conservativeResize(checkpoints.left.rows(), checkpoints.left.cols() - 1);

    checkpoints.right.leftCols(checkpoints.right.cols() - 1) = checkpoints.right.rightCols(checkpoints.right.cols() - 1);
    checkpoints.right.conservativeResize(checkpoints.right.rows(), checkpoints.right.cols() - 1);
    
    checkpoints.center.leftCols(checkpoints.center.cols() - 1) = checkpoints.center.rightCols(checkpoints.center.cols() - 1);
    checkpoints.center.conservativeResize(checkpoints.center.rows(), checkpoints.center.cols() - 1);
    
    checkpoints.yaw.leftCols(checkpoints.yaw.cols() - 1) = checkpoints.yaw.rightCols(checkpoints.yaw.cols() - 1);
    checkpoints.yaw.conservativeResize(checkpoints.yaw.rows(), checkpoints.yaw.cols() - 1);

    checkpoints.forward_vector.leftCols(checkpoints.forward_vector.cols() - 1) = checkpoints.forward_vector.rightCols(checkpoints.forward_vector.cols() - 1);
    checkpoints.forward_vector.conservativeResize(checkpoints.forward_vector.rows(), checkpoints.forward_vector.cols() - 1);

    return	checkpoints;
}

int add_turn(CheckPoints& checkpoints, const double& phi, const double& L, const double& width)
{
	double kappa = (phi * (2 * PI)) / L;

	const size_t N = 40;

	double ds = L / N;

	for (size_t i = 1; i <= N; ++i)
	{
		checkpoints.yaw.conservativeResize(checkpoints.yaw.rows(), checkpoints.yaw.cols() + 1);
		checkpoints.yaw(0, checkpoints.yaw.cols() - 1) = checkpoints.yaw(0, checkpoints.yaw.cols() - 2) + kappa * ds;

		double c = std::cos(checkpoints.yaw(0, checkpoints.yaw.cols() - 1));
		double s = std::sin(checkpoints.yaw(0, checkpoints.yaw.cols() - 1));
		Eigen::Matrix<double, 2, 1> f;
		f << c, s;
		Eigen::Matrix<double, 2, 1> n;
		n << -s, c;

		checkpoints.center.conservativeResize(checkpoints.center.rows(), checkpoints.center.cols() + 1);
		checkpoints.center.rightCols(1) = checkpoints.center.col(checkpoints.center.cols() - 2) + f * ds;

		checkpoints.left.conservativeResize(checkpoints.left.rows(), checkpoints.left.cols() + 1);
		checkpoints.left.rightCols(1) = checkpoints.center.col(checkpoints.center.cols() - 1) + n * width / 2;

		checkpoints.right.conservativeResize(checkpoints.right.rows(), checkpoints.right.cols() + 1);
		checkpoints.right.rightCols(1) = checkpoints.center.col(checkpoints.center.cols() - 1) - n * width / 2;	

		checkpoints.forward_vector.conservativeResize(checkpoints.forward_vector.rows(), checkpoints.forward_vector.cols() + 1);
		checkpoints.forward_vector.rightCols(1) = f;

	}

	return 1;
}