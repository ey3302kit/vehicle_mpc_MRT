#pragma once
#include <float.h>
#include <Eigen/Dense>
#include <ct/optcon/optcon.h>

#include "OptVectorDmsVehicle.h"
#include "testTrack1.h"
#include "get_AccelerationConstraints.h"
#include "get_CheckpointIndices.h"

// double a = std::numeric_limits<double>::min();
// double a = std::numeric_limits<double>::max();

namespace ct{
namespace optcon{
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class InequalConstraintsVehicle : public tpl::DiscreteConstraintBase<SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef tpl::DiscreteConstraintBase<SCALAR> Base;
	typedef ct::core::StateVector<STATE_DIM, SCALAR> state_vector_t;
	typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;

	typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
	typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;

	typedef ct::core::tpl::TimeArray<SCALAR> time_array_t;
	typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;

	InequalConstraintsVehicle(std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM>> w,
		const size_t & N)
		: w_(w),
		N_(N),
		constraintsCount_(18 * (N + 1)),
		nonZeroJacCount_(38 * (N + 1))
	{
		discreteUpperBound_.resize(constraintsCount_);
		discreteLowerBound_.resize(constraintsCount_);  

        discreteUpperBound_.setConstant(DBL_MAX);
        discreteLowerBound_.setConstant(-DBL_MAX);  

        n << 0, -1, 1, 0;

        this->initializeSparsityPatternofFirstIteration();
        this->generate();
    }

	~InequalConstraintsVehicle() override = default;

    /*
        evalute the constraints for all variables in optVector(for different shoting intervals)
    */
    VectorXs eval() override
    {
        VectorXs result(constraintsCount_);

        size_t Idx(0);

        double xi(w_->getXi());

        for (size_t i = 0; i < N_ + 1; ++i)
        {
            auto left_unit_vector =  n * checkpoint.forward_vector.col(CheckpointIndices_(i));

            auto x_i = w_->getOptimizedState(i);
            auto u_i = w_->getOptimizedControl(i);

            result(Idx) = x_i.head(2).transpose() * left_unit_vector - xi;
            ++Idx;
            result(Idx) = x_i.head(2).transpose() * (-left_unit_vector) - xi;
            ++Idx;

            for (size_t j = 0; j < 16; ++j)
            {
                result(Idx) = A_Vector[j] * u_i;

                ++Idx;
            }
        }

    	return result; 
    }


    VectorXs evalSparseJacobian() override
    {
        discreteJac_.resize(nonZeroJacCount_);

        size_t Idx(0);

        for (size_t i = 0; i < N_ + 1; ++i)
        {
            auto left_unit_vector = n * checkpoint.forward_vector.col(CheckpointIndices_(i));

            discreteJac_(Idx) = left_unit_vector(0);
            ++Idx;
            discreteJac_(Idx) = left_unit_vector(1);
            ++Idx;

            discreteJac_(Idx) = -left_unit_vector(0);
            ++Idx;
            discreteJac_(Idx) = -left_unit_vector(1);
            ++Idx;

            for (size_t j = 0; j < 16; j++)
            {
                discreteJac_(Idx) = A_Vector[j](0);
                ++Idx;
                discreteJac_(Idx) = A_Vector[j](1);
                ++Idx;
            }
        }

        discreteJac_.segment(Idx, 2 * (N_ + 1)).setConstant(-1.0);

        return discreteJac_;
    }

    size_t getConstraintSize() override { return constraintsCount_; }

    size_t getNumNonZerosJacobian() override { return nonZeroJacCount_; }
    
    void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        iRow_vec = this->iRow_vec_;
        jCol_vec = this->jCol_vec_;
    }

    VectorXs getLowerBound() override
    {
        return discreteLowerBound_;
    }

    VectorXs getUpperBound() override
    {
        return discreteUpperBound_;
    }

    // update the constraints after find the optimal trajectory after one MPC loop
    void updatePreviousOptimalStates(const state_vector_array_t & PreviousOptimalStates)
    {
        CheckpointIndices_ = get_CheckpointIndices(PreviousOptimalStates, checkpoint);

        size_t Idx(0);
        
        
        for (size_t i = 0; i < N_ + 1; ++i)
        {

            auto left_unit_vector =  n * checkpoint.forward_vector.col(CheckpointIndices_(i));

            discreteUpperBound_(Idx) = checkpoint.left.col(CheckpointIndices_(i)).transpose() * left_unit_vector;
            ++Idx;
            discreteUpperBound_(Idx) = checkpoint.right.col(CheckpointIndices_(i)).transpose() * (-left_unit_vector);
            ++Idx;

            for (size_t j = 0; j < 16; ++j)
            {
                get_AccelerationConstraints(A, b, PreviousOptimalStates[i], j);

                A_Vector.push_back(A);
                b_Vector.push_back(b);

                discreteUpperBound_(Idx) = b;
                ++Idx;
            }
        }
    }
    
private:
	std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> w_;

	// store the coefficientmatrix of input constraints
    std::vector<Eigen::Matrix<double, 1, 2>> A_Vector;
    std::vector<double> b_Vector;
    Eigen::Matrix<double, 1, 2> A;
    double b;

    size_t N_;

    size_t constraintsCount_;

    VectorXs discreteLowerBound_;
    VectorXs discreteUpperBound_;

    VectorXs discreteJac_;
    VectorXi iRow_vec_;
    VectorXi jCol_vec_;

    Eigen::Matrix<int, 36, 1> Row_OneIteration;
    Eigen::Matrix<int, 36, 1> Col_OneIteration;    

    // store the indices of the previous optimal states
    Eigen::Matrix<size_t, Eigen::Dynamic, 1> CheckpointIndices_;

    size_t nonZeroJacCount_;

    const double L = 3.0;
    const double A_MAX = 40.0;
    Eigen::Matrix<double, 2, 2> n;

    const CheckPoints checkpoint = testTrack1();

    void initializeSparsityPatternofFirstIteration()
    {
        size_t Idx(0);
        for (int i = 0; i < 18; ++i) 
        {                
            this->Row_OneIteration.segment(Idx, 2) << i, i;
            Idx += 2;
        }

        Idx = 0;
        this->Col_OneIteration.segment(Idx, 2) << 0, 1;
        Idx += 2;
        this->Col_OneIteration.segment(Idx, 2) << 0, 1;
        Idx += 2;
        for (size_t i = 0; i < 16; i++)
        {
            this->Col_OneIteration.segment(Idx, 2) << 4, 5;
            Idx += 2;
        } 
    }

    void genSparsityPatternofOneIteration(int iteration, Eigen::Matrix<int, 36, 1> & current_Row, Eigen::Matrix<int, 36, 1> & current_Col)
    {
        current_Row = iteration * 18 + this->Row_OneIteration.array();
        current_Col = iteration * (STATE_DIM + CONTROL_DIM) + this->Col_OneIteration.array();
    }

    void generate()
    {   
        Eigen::Matrix<int, 36, 1> current_Row_;
        Eigen::Matrix<int, 36, 1> current_Col_;

        this->iRow_vec_.resize(nonZeroJacCount_);
        this->jCol_vec_.resize(nonZeroJacCount_);

        size_t Idx(0);
        for (int i = 0; i < N_ + 1; ++i)
        {
            this->genSparsityPatternofOneIteration(i, current_Row_, current_Col_);
            this->iRow_vec_.segment(Idx, 36) = current_Row_;
            this->jCol_vec_.segment(Idx, 36) = current_Col_;

            Idx += 36;
        }

        for (int i = 0; i < N_ + 1; ++i)
        {
            this->iRow_vec_.segment(Idx, 2) << 0 + i * 18, 1 + i * 18;
            this->jCol_vec_.segment(Idx, 2) << (STATE_DIM + CONTROL_DIM) * (N_ + 1), (STATE_DIM + CONTROL_DIM) * (N_ + 1);

            Idx += 2;
        }
    }

};
}
}