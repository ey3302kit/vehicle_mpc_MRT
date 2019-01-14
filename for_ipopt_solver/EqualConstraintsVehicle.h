#pragma once
#include <limits>
#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "OptVectorDmsVehicle.h"

// double a = std::numeric_limits<double>::min();
// double a = std::numeric_limits<double>::max();

namespace ct{
namespace optcon{
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class EqualConstraintsVehicle : public tpl::DiscreteConstraintBase<SCALAR>
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

	EqualConstraintsVehicle(std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM>> w,
		const size_t & N)
		: w_(w),
		N_(N),
		constraintsCount_(4 * (N + 1) + 2),
		nonZeroJacCount_(8 + 14 * N + 2)
	{
		discreteUpperBound_.resize(constraintsCount_);
		discreteLowerBound_.resize(constraintsCount_);  

        discreteUpperBound_.setConstant(0);
        discreteLowerBound_.setConstant(0);  

        A << 1, 0, 0.1, 0,
            0, 1, 0, 0.1,
            0, 0, 1, 0,
            0, 0, 0, 1;

        B << 0.005, 0,
            0, 0.005,
            0.1, 0,
            0, 0.1;

        this->initializeSparseofFirstConstraint();
        this->generateSparse();
    }

	~EqualConstraintsVehicle() override = default;

    /*
        evalute the constraints for all variables in optVector(for different shoting intervals)
    */
    VectorXs eval() override
    {
        VectorXs result(constraintsCount_);

        size_t Idx(0);

        result.segment(Idx, STATE_DIM) = w_->getOptimizedState(0) - B * w_->getOptimizedControl(0);
        Idx += STATE_DIM;

        for (size_t i = 1; i < N_ + 1; ++i)
        {
            result.segment(Idx, STATE_DIM) = w_->getOptimizedState(i) - B * w_->getOptimizedControl(i)  - A * w_->getOptimizedState(i - 1);
            Idx += STATE_DIM;
        }

        result.segment(Idx, 2) = w_->getOptimizedState(N_).segment(2, 2); 

    	return result; 
    }


    VectorXs evalSparseJacobian() override
    {
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
    void updateInitialState(const state_vector_t & x0)
    {   
            discreteUpperBound_.segment(0, STATE_DIM) = A * x0;
            discreteLowerBound_.segment(0, STATE_DIM) = A * x0;
    }
    
private:
	std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> w_;
    size_t N_;

    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4 ,2> B;

    size_t constraintsCount_;
    size_t nonZeroJacCount_;

    // Sparse of one iteration
    Eigen::Matrix<double, 14, 1> jac_OneIteration;
    Eigen::Matrix<int, 14, 1> Row_OneIteration;
    Eigen::Matrix<int, 14, 1> Col_OneIteration;

    // Sparse
    VectorXs discreteJac_;
    Eigen::VectorXi iRow_vec_; 
    Eigen::VectorXi jCol_vec_;

    VectorXs discreteLowerBound_;
    VectorXs discreteUpperBound_;


    void initializeSparseofFirstConstraint()
    {
        this->jac_OneIteration << -1, -0.1, 1, -0.005, 
            -1, -0.1, 1, -0.005, 
            -1, 1, -0.1,
            -1, 1, -0.1;

        this->Row_OneIteration << 4, 4, 4, 4,
            5, 5, 5, 5,
            6, 6, 6,
            7, 7, 7;

        this->Col_OneIteration << 0, 2, 6, 10,
            1, 3, 7, 11,
            2, 8, 10,
            3, 9, 11;

    }

    void generateSparseofOneIteration(int iteration, Eigen::Matrix<double, 14, 1> & current_jac, Eigen::Matrix<int, 14, 1> & current_Row, Eigen::Matrix<int, 14, 1> & current_Col)
    {
        current_jac = this->jac_OneIteration;

        current_Row = iteration * STATE_DIM + this->Row_OneIteration.array();

        current_Col = iteration * (STATE_DIM + CONTROL_DIM) + this->Col_OneIteration.array();
    }

    void generateSparse()
    {
        Eigen::Matrix<double, 14, 1> current_jac_;
        Eigen::Matrix<int, 14, 1> current_Row_;
        Eigen::Matrix<int, 14, 1> current_Col_;

        this->discreteJac_.resize(nonZeroJacCount_);
        this->iRow_vec_.resize(nonZeroJacCount_);
        this->jCol_vec_.resize(nonZeroJacCount_);

        this->discreteJac_.segment(0, 8) << 1, -0.005, 1, -0.005, 1, -0.1, 1, -0.1;
        this->iRow_vec_.segment(0, 8) << 0, 0, 1, 1, 2, 2, 3, 3;
        this->jCol_vec_.segment(0, 8) << 0, 4, 1, 5, 2, 4, 3, 5;


        size_t Idx(8);
        for (int i = 0; i < N_; ++i)
        {
            this->generateSparseofOneIteration(i, current_jac_, current_Row_, current_Col_);

            this->discreteJac_.segment(Idx, 14) = current_jac_;
            this->iRow_vec_.segment(Idx, 14) = current_Row_;
            this->jCol_vec_.segment(Idx, 14) = current_Col_;

            Idx += 14;
        }

        this->discreteJac_(Idx) = 1;
        this->discreteJac_(Idx + 1) = 1;
        this->iRow_vec_(Idx) = STATE_DIM * (N_ + 1);
        this->iRow_vec_(Idx + 1) = STATE_DIM * (N_ + 1) + 1;
        this->jCol_vec_(Idx) = (STATE_DIM + CONTROL_DIM) * (N_ + 1) - 4;
        this->jCol_vec_(Idx + 1) = (STATE_DIM + CONTROL_DIM) * (N_ + 1) - 3;
    }
};
}
}

