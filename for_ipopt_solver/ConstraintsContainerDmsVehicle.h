#pragma once

#include <Eigen/Dense>
#include <ct/optcon/optcon.h>

#include "OptVectorDmsVehicle.h"
#include "BoundConstraintsVehicle.h"
#include "EqualConstraintsVehicle.h"
#include "InequalConstraintsVehicle.h"

namespace ct{
namespace optcon{

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstraintsContainerDmsVehicle : public tpl::DiscreteConstraintContainerBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;

    typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
    typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
    typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
    typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
    typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;
    typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;

    ConstraintsContainerDmsVehicle(std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> w,
    	const size_t & N)
    {
    	boundConstraint_ = std::shared_ptr<BoundConstraintsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>>(
    		new BoundConstraintsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>(w, N));
    	equalConstraint_ = std::shared_ptr<EqualConstraintsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>>(
    		new EqualConstraintsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>(w, N));
    	inequalConstraint_ = std::shared_ptr<InequalConstraintsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>>(
    		new InequalConstraintsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>(w, N));

    	this->constraints_.push_back(boundConstraint_);
    	this->constraints_.push_back(equalConstraint_);
    	this->constraints_.push_back(inequalConstraint_);
    }

    ~ConstraintsContainerDmsVehicle() override = default;

	void prepareEvaluation() override
	{
	}

    void prepareJacobianEvaluation() override
    {
    }

    void updateConstraints(const state_vector_array_t & PreviousOptimalStates, const state_vector_t & x0)
    {
    	boundConstraint_->updatePreviousOptimalStates(PreviousOptimalStates);
    	equalConstraint_->updateInitialState(x0);
    	inequalConstraint_->updatePreviousOptimalStates(PreviousOptimalStates);
    }

private:
	std::shared_ptr<BoundConstraintsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> boundConstraint_;
	std::shared_ptr<EqualConstraintsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> equalConstraint_;
	std::shared_ptr<InequalConstraintsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> inequalConstraint_;
};
}
}