#pragma once
#include <ct/optcon/optcon.h>
#include <Eigen/Dense>
#include "TestTrack.h"

namespace ct{
namespace optcon{
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class TerminalLinearTerm_Vehicle : public TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_double_t;

    TerminalLinearTerm_Vehicle(
    	std::shared_ptr<TestTrack<STATE_DIM, CONTROL_DIM>> test_track,
    	std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>> previous_optimal_states);

    TerminalLinearTerm_Vehicle(const TerminalLinearTerm_Vehicle& arg);

    virtual TerminalLinearTerm_Vehicle* clone() const override
    {
		return new TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(*this);
    }

    ~TerminalLinearTerm_Vehicle() = default;

    virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
    	const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
    	const SCALAR& t) override;

    virtual core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(
    	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    	const SCALAR_EVAL& t) override;

    virtual state_matrix_t stateSecondDerivative(
    	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    	const SCALAR_EVAL& t) override;

    virtual core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivative(
    	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    	const SCALAR_EVAL& t) override;

    virtual control_matrix_t controlSecondDerivative(
    	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    	const SCALAR_EVAL& t) override;

    virtual control_state_matrix_t stateControlDerivative(
    	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
    	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
    	const SCALAR_EVAL& t) override;

private:
	std::shared_ptr<TestTrack<STATE_DIM, CONTROL_DIM>> test_track_;
	std::shared_ptr< ct::core::StateVectorArray<STATE_DIM, SCALAR> > previous_optimal_states_;
};

#include "TerminalLinearTerm_Vehicle-impl.cpp"
}
}