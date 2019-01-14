#pragma once 
#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "OptVectorDmsVehicle.h"
#include "ConstraintsContainerDmsVehicle.h"
#include "CostEvaluatorVehicle.h"

namespace ct{
namespace optcon{

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DmsProblemVehicle : public tpl::Nlp<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;
    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

    typedef ContinuousOptConProblem<STATE_DIM, CONTROL_DIM, SCALAR> OptConProblem_t;

    DmsProblemVehicle(const size_t & N, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> & R, const double & q)
    {	
    	// define the optVec
    	size_t wLength = (N + 1) * (STATE_DIM + CONTROL_DIM) + 1;
    	this->optVariables_ = std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>>(
            new OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>(wLength, N));

    	optVariablesDmsVehicle_ = std::static_pointer_cast<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>>(this->optVariables_);

    	// define the objective function
    	this->costEvaluator_ = std::shared_ptr<CostEvaluatorVehicle<STATE_DIM, CONTROL_DIM, SCALAR>>(
            new CostEvaluatorVehicle<STATE_DIM, CONTROL_DIM, SCALAR>(optVariablesDmsVehicle_, N, q, R));

    	costEvaluatorDmsVeicle_ = std::static_pointer_cast<CostEvaluatorVehicle<STATE_DIM, CONTROL_DIM, SCALAR>>(this->costEvaluator_);

    	// define the constraints
    	this->constraints_ = std::shared_ptr<ConstraintsContainerDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>>(
    		new ConstraintsContainerDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>(optVariablesDmsVehicle_, N));

    	constraintDmsVehicle_ = std::static_pointer_cast<ConstraintsContainerDmsVehicle<STATE_DIM, CONTROL_DIM ,SCALAR>>(this->constraints_);

    	this->optVariables_->resizeConstraintVars(this->getConstraintsCount());
    }

    ~DmsProblemVehicle() = default;

    void updateProblem() override
    {
    }

    void updateProblem(const state_vector_array_t & PreviousOptimalStates, const state_vector_t & x0)
    {
    	constraintDmsVehicle_->updateConstraints(PreviousOptimalStates, x0);
    	costEvaluatorDmsVeicle_->updateCost(PreviousOptimalStates);
    }

    const state_vector_array_t& getStateSolution() { return optVariablesDmsVehicle_->getOptimizedStates(); }
    const control_vector_array_t& getInputSolution() { return optVariablesDmsVehicle_->getOptimizedInputs(); }
    const double getXi() { return optVariablesDmsVehicle_->getXi(); }

    const control_vector_t & getControl() { return optVariablesDmsVehicle_->getOptimizedControl(0); }


    void setInitialGuess(const state_vector_array_t& x_init_guess,
    	const control_vector_array_t& u_init_guess,
        const double & xi_init_guess)
    {
        optVariablesDmsVehicle_->setInitGuess(x_init_guess, u_init_guess, xi_init_guess);
    }


private:
	std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> optVariablesDmsVehicle_;
	std::shared_ptr<ConstraintsContainerDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> constraintDmsVehicle_;
	std::shared_ptr<CostEvaluatorVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> costEvaluatorDmsVeicle_;
};
}
}