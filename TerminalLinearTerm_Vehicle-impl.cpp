template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TerminalLinearTerm_Vehicle(
	std::shared_ptr<TestTrack<STATE_DIM, CONTROL_DIM>> test_track,
	std::shared_ptr<ct::core::StateVectorArray<STATE_DIM, SCALAR>> previous_optimal_states) : test_track_(test_track), previous_optimal_states_(previous_optimal_states)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename	SCALAR>
TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::TerminalLinearTerm_Vehicle(
	const TerminalLinearTerm_Vehicle& arg) : test_track_(arg.test_track_), previous_optimal_states_(arg.previous_optimal_states_)
{}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
SCALAR TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evaluate(
	const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
	const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
	const SCALAR& t)
{
	Eigen::Matrix<double, 2, 1> forward_vector_;

	test_track_->get_ForwardVector(forward_vector_, (*previous_optimal_states_).back());

	return (-forward_vector_.transpose() * x.segment(0, 2));
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
core::StateVector<STATE_DIM, SCALAR_EVAL> TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateDerivative(
	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
	const SCALAR_EVAL& t)
{
	ct::core::StateVector<STATE_DIM, SCALAR_EVAL> jac_state(Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1>::Zero());
	Eigen::Matrix<double, 2, 1> forward_vector_;
	test_track_->get_ForwardVector(forward_vector_, (*previous_optimal_states_).back());
	jac_state(0) = forward_vector_(0);
	jac_state(1) = forward_vector_(1);

	return (-jac_state);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::state_matrix_t
TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateSecondDerivative(
	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
	const SCALAR_EVAL& t)
{
	return state_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> 
TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlDerivative(
	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
	const SCALAR_EVAL& t)
{
	return ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_matrix_t
TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::controlSecondDerivative(
	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
	const SCALAR_EVAL& t)
{
	return control_matrix_t::Zero();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
typename TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::control_state_matrix_t
TerminalLinearTerm_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::stateControlDerivative(
	const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
	const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
	const SCALAR_EVAL& t)
{
	return control_state_matrix_t::Zero();
}
