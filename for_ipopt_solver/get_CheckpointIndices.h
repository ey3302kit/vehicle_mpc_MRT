#pragma once
#include <Eigen/Dense>
#include <ct/core/core.h>

#include "testTrack1.h"

Eigen::Matrix<size_t, Eigen::Dynamic, 1> get_CheckpointIndices(const ct::core::StateVectorArray<4, double> & state_vector_array, 
	const CheckPoints & Checkpoint)
{

	Eigen::Matrix<size_t, Eigen::Dynamic, 1> CheckpointIndices;
	CheckpointIndices.resize(state_vector_array.size());

	size_t Idx = 0;
	
	for (auto state_vector : state_vector_array)
	{	

		const size_t n_Checkpoint = Checkpoint.center.cols();

		Eigen::MatrixXd differenceMatrix = (Checkpoint.center - state_vector.topRows(2).replicate(1,n_Checkpoint));
		differenceMatrix = differenceMatrix.cwiseProduct(differenceMatrix);

		Eigen::MatrixXd sumdifferenceMatrix;
		sumdifferenceMatrix = differenceMatrix.colwise().sum();

		size_t maxCol, maxRow;
		sumdifferenceMatrix.minCoeff(&maxRow, &maxCol);


		CheckpointIndices(Idx) = maxCol;
		++Idx;
/*		Eigen::MatrixXd n(2, 2);
		n << 0, -1, 1, 0;

		normal_vector = n * Checkpoint.forward_vector.col(maxCol);
		left_track << Checkpoint.left.col(maxCol)(0), Checkpoint.left.col(maxCol)(1);
		right_track << Checkpoint.right.col(maxCol)(0),Checkpoint.right.col(maxCol)(1);*/
	}
	
	return CheckpointIndices;
} 


/*int main(int argc, char const *argv[])
{
	while (true)
	{	
		Eigen::Matrix<double, 5, 1> state_Matrix;
		std::cin >> state_Matrix(0) >> state_Matrix(1) >> state_Matrix(2) >> state_Matrix(3);

		Eigen::Matrix<double, 2, 1> normal_vector;
		Eigen::Matrix<double, 2, 1> left_track;
		Eigen::Matrix<double, 2, 1> right_track;
		for (size_t i = 1; i <= 16; ++i)
		{
			get_CheckpointIndices(state_Matrix, normal_vector, left_track, right_track);

			std::cout << i << std::endl;
			std::cout << normal_vector << std::endl;
			std::cout << left_track << std::endl;
			std::cout << right_track << std::endl;
			std::cout << std::endl;

		}

	}
	return 0;
}*/