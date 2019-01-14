#pragma once
#include <float.h>
#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "OptVectorDmsVehicle.h"

// double a = std::numeric_limits<double>::min();
// double a = std::numeric_limits<double>::max();

namespace ct{
namespace optcon{
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class BoundConstraintsVehicle : public tpl::DiscreteConstraintBase<SCALAR>
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

	BoundConstraintsVehicle(std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM>> w,
		const size_t & N)
		: w_(w),
		N_(N),
		constraintsCount_((STATE_DIM + CONTROL_DIM) * (N + 1) + 1),
		nonZeroJacCount_((STATE_DIM + CONTROL_DIM) * (N + 1) + 1)
	{
		discreteUpperBound_.resize(constraintsCount_);
		discreteLowerBound_.resize(constraintsCount_);

		discreteUpperBound_.setConstant(DBL_MAX);
		discreteLowerBound_.setConstant(-DBL_MAX);

    }

	~BoundConstraintsVehicle() override = default;

    /*
        evalute the constraints for all variables in optVector(for different shoting intervals)
    */
    VectorXs eval() override
    {
    	return (w_->getX());
    }


    VectorXs evalSparseJacobian() override
    {
        discreteJac_.resize(nonZeroJacCount_);

        discreteJac_.setConstant(1);

        return discreteJac_;
    }

    size_t getConstraintSize() override { return constraintsCount_; }

    size_t getNumNonZerosJacobian() override { return nonZeroJacCount_; }
    
    void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {

    	Eigen::VectorXi discreteIRow_(nonZeroJacCount_);
    	Eigen::VectorXi discreteJCol_(nonZeroJacCount_);

    	for (size_t Idx = 0; Idx < nonZeroJacCount_; ++Idx)
    	{
    		discreteIRow_(Idx) = Idx;
    		discreteJCol_(Idx) = Idx;
    	}


        iRow_vec = discreteIRow_;
        jCol_vec = discreteJCol_;
    }

    VectorXs getLowerBound() override
    {
        return discreteLowerBound_;
    }

    VectorXs getUpperBound() override
    {
        return discreteUpperBound_;
    }

    void updatePreviousOptimalStates(const state_vector_array_t & PreviousOptimalStates)
    {

    	size_t Idx(0);

		for (size_t i = 0; i < N_ + 1; ++i)
		{
			discreteLowerBound_(Idx) = PreviousOptimalStates[i](0) - L; //pix - L;
			discreteUpperBound_(Idx) = PreviousOptimalStates[i](0) + L;//pix + L;

			++Idx;
			discreteLowerBound_(Idx) = PreviousOptimalStates[i](1) - L;//pix - L;
			discreteUpperBound_(Idx) = PreviousOptimalStates[i](1) + L;//pix + L;

			Idx += 3;
			discreteLowerBound_(Idx) = -A_MAX;
			discreteUpperBound_(Idx) = A_MAX;

			++Idx;
			discreteLowerBound_(Idx) = -A_MAX;
			discreteUpperBound_(Idx) = A_MAX;

			++Idx;
		}

		discreteLowerBound_(Idx) = 0;
    }
    
private:
	std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> w_;

    size_t N_;

    size_t constraintsCount_;

    VectorXs discreteLowerBound_;
    VectorXs discreteUpperBound_;

    VectorXs discreteJac_;

    size_t nonZeroJacCount_;

    const double L = 3.0;
    const double A_MAX = 40.0;

};
}
}