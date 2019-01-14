#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

/*int main(int argc, char const *argv[])
{
	Eigen::Matrix<double, 2, Eigen::Dynamic> a_lateral_max_list;
	Eigen::Matrix<double, 2, Eigen::Dynamic> a_forward_max_list;
	Eigen::Matrix<double, 2, Eigen::Dynamic> a_backward_max_list;

	get_Acceleration_Limits(
	a_lateral_max_list,
	a_forward_max_list,
	a_backward_max_list);

	std::cout << a_lateral_max_list <<std::endl;
	std::cout << a_forward_max_list <<std::endl;
	std::cout << a_backward_max_list <<std::endl;
	return 0;
}*/

class AccelerationLimits
{
public:
	// constructor
	AccelerationLimits() : delta_x_(0.01)
	{
		xValue.resize(5);
		yValue.resize(5);
		xValue << 0.0, 10.0, 43.0, 52.0, 200.0;
		yValue << 1.0, 14.0, 28.0, 33.0, 33.0;
		this->interpl(xValue, yValue, delta_x_, a_lateral_max_list);

		xValue.resize(8);
		yValue.resize(8);
		xValue << 0.0, 10.0, 20.0, 35.0, 52.0, 79.0, 83.0, 200.0;
		yValue << 2.0, 13.0, 18.0, 18.0, 15.0, 6.0, 1.0, 1.0;
		this->interpl(xValue, yValue, delta_x_, a_forward_max_list);

		xValue.resize(6);
		yValue.resize(6);
		xValue << 0.0, 30.0, 40.0, 52.0, 76.0, 200.0;
		yValue << 11.0, 13.0, 24.0, 30.0, 40.0, 40.0;
		this->interpl(xValue, yValue, delta_x_, a_backward_max_list);
	}

	~AccelerationLimits() {}

	int get_AccelerationLimits(Eigen::Vector3d & Limits, const double & v)
	{	
		size_t v_index = std::min(12001, int(std::round(100 * v)));
		
		Limits(0) = this->a_lateral_max_list.col(v_index)(1);
		Limits(1) = this->a_forward_max_list.col(v_index)(1);
		Limits(2) = this->a_backward_max_list.col(v_index)(1);

		return 0;
	}

private:
	Eigen::Matrix<double, 2, Eigen::Dynamic> a_lateral_max_list;
	Eigen::Matrix<double, 2, Eigen::Dynamic> a_forward_max_list;
	Eigen::Matrix<double, 2, Eigen::Dynamic> a_backward_max_list;

	Eigen::VectorXd xValue;
	Eigen::VectorXd yValue;

	double delta_x_;
	double delta_y_;

	void push_back(Eigen::Matrix<double, 2, Eigen::Dynamic>& m, Eigen::Vector2d vectorAppend)
	{
		m.conservativeResize(m.rows(), m.cols() + 1);
		m.rightCols(1) = vectorAppend;
	}

	int interpl(
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

				//avoid comparision between double
				size_t n = (xValue(i+1) - xValue(i)) / delta_x_;

				for (size_t j = 0; j < n && (vectorAppend(0) <= 120);j++)
				{
					vectorAppend << vectorAppend(0) + delta_x_, vectorAppend(1) + delta_y_;
					push_back(InterpolationResult, vectorAppend);
				}
			}
			//std::cout << InterpolationResult.cols() <<std::endl;
			return 0;
		}
	}

};