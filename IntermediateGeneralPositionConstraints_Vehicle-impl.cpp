template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::IntermediateGeneralPositionConstraints_Vehicle(
	std::shared_ptr< ct::core::StateVectorArray<STATE_DIM, SCALAR> > previous_optimal_states,
	std::shared_ptr<TestTrack<STATE_DIM, CONTROL_DIM>> test_track,
	const double& delta_t) :
	previous_optimal_states_(previous_optimal_states),
	test_track_(test_track),
	delta_t_(delta_t)
{
	this->lb_.resize(getConstraintSize());
	this->ub_.resize(getConstraintSize());

	this->ub_.segment(0, 2).setConstant(SCALAR(0.));
	//this->lb_.segment(0, 2).setConstant(-std::numeric_limits<SCALAR>::infinity());
	this->lb_.segment(0, 2).setConstant(-1e5);
	this->ub_.segment(2, 2).setConstant(L);
	this->lb_.segment(2, 2).setConstant(-L);

	initialize();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::IntermediateGeneralPositionConstraints_Vehicle(
	const IntermediateGeneralPositionConstraints_Vehicle& arg) :
	previous_optimal_states_(arg.previous_optimal_states_),
	test_track_(arg.test_track_),
	delta_t_(arg.delta_t_),
	L(arg.L),
	jac_rows_state(arg.jac_rows_state),
	jac_cols_state(arg.jac_cols_state),
	jac_rows_input(arg.jac_rows_input),
	jac_cols_input(arg.jac_cols_input)
{
	this->lb_ = arg.lb_;
	this->ub_ = arg.ub_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	VectorXs val;
	size_t i = size_t(t / delta_t_);

	Eigen::Matrix<double, 2, 1> left;
	Eigen::Matrix<double, 2, 1> right;
	Eigen::Matrix<double, 2, 1> forward_vector;
	Eigen::Matrix<double, 2, 2> rotate;
	double p_x, p_y;
	rotate << 0, -1, 1, 0;

	test_track_->get_Track(left, right, forward_vector, (*previous_optimal_states_)[i]);

	val.resize(getConstraintSize());

	val(0) = ((x.segment(0, 2) - left).transpose() * rotate * forward_vector)(0) - u(2);
	val(1) = ((x.segment(0, 2) - right).transpose() * rotate * (-forward_vector))(0) - u(2);
	val(2) = x(0) - ((*previous_optimal_states_)[i])(0);
	val(3) = x(1) - ((*previous_optimal_states_)[i])(1);

	return val;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintSize() const
{
	return 4;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getLowerBound() const
{
	return this->lb_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getUpperBound() const
{
	return this->ub_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianState(
	const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	MatrixXs jac(getConstraintSize(), STATE_DIM);
	jac.setZero();

	VectorXs sparse(jacobianStateSparse(x, u ,t));

	for (size_t i = 0; i < getNumNonZerosJacobianState(); i++)
		jac(jac_rows_state(i), jac_cols_state(i)) = sparse(i);

	return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInput(
	const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	MatrixXs jac(getConstraintSize(), CONTROL_DIM);
	jac.setZero();

	VectorXs sparse(jacobianInputSparse(x, u, t));

	for (size_t i = 0; i < getNumNonZerosJacobianInput(); i++)
		jac(jac_rows_input(i), jac_cols_input(i)) = sparse(i);

	return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
	return (jac_rows_state.size());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianInput() const
{
	return (jac_rows_input.size());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparse(
	const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	VectorXs sparse(getNumNonZerosJacobianState());

	size_t i = size_t(t / delta_t_);

	Eigen::Matrix<double, 2, 1> forward_vector;
	Eigen::Matrix<double, 2, 2> rotate;
	rotate << 0, -1, 1, 0;

	test_track_->get_ForwardVector(forward_vector, (*previous_optimal_states_)[i]);

	Eigen::Matrix<double, 2, 1> n;
	n = rotate * forward_vector;

	sparse << n(0), n(1), -n(0), -n(1), 1, 1;

	return sparse;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparse(
	const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	VectorXs sparse(getNumNonZerosJacobianInput());
	sparse << -1, -1;
	return sparse;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::sparsityPatternState(
	Eigen::VectorXi& rows, Eigen::VectorXi& cols)
{
	rows.resize(jac_rows_state.size());
	cols.resize(jac_cols_state.size());

	rows = jac_rows_state;
	cols = jac_cols_state;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::sparsityPatternInput(
	Eigen::VectorXi& rows, Eigen::VectorXi& cols)
{
	rows.resize(jac_rows_input.size());
	cols.resize(jac_cols_input.size());

	rows = jac_rows_input;
	cols = jac_cols_input;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> 
IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::CppadCg(
	const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
	const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
	ct::core::ADCGScalar t)
{
	Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> val(getConstraintSize());

	size_t i = CppAD::Value(t).getValue() / delta_t_;

	Eigen::Matrix<double, 2, 1> left(Eigen::Matrix<double, 2, 1>::Ones());
	Eigen::Matrix<double, 2, 1> right(Eigen::Matrix<double, 2, 1>::Ones());
	Eigen::Matrix<double, 2, 1> forward_vector(Eigen::Matrix<double, 2, 1>::Ones());
	Eigen::Matrix<double, 2, 2> rotate;
	double p_x(1.), p_y(1.);
	rotate << 0, -1, 1, 0;

	//test_track_->get_Track(left, right, forward_vector, p_x, p_y, (*previous_optimal_states_)[i]);

	val << ((x.segment(0, 2) - left.template cast<ct::core::ADCGScalar>()).transpose() * (rotate * forward_vector).template cast<ct::core::ADCGScalar>())(0) - u(2),
	((x.segment(0, 2) - left.template cast<ct::core::ADCGScalar>()).transpose() * (-rotate * forward_vector).template cast<ct::core::ADCGScalar>())(0) - u(2),
	x(0) - ct::core::ADCGScalar(p_x),
	x(1) - ct::core::ADCGScalar(p_y);

	return val;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void IntermediateGeneralPositionConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::initialize()
{
	typename JacCG::FUN_TYPE_CG f_constraint_;

	f_constraint_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM, 1>& stateinput)
	{
		return CppadCg(stateinput.segment(0, STATE_DIM), stateinput.segment(STATE_DIM, CONTROL_DIM), CGScalar(0.0));
	};

	std::shared_ptr<JacCG> constraint_Codegen_(
		new JacCG(f_constraint_, STATE_DIM + CONTROL_DIM, this->getConstraintSize()));

	ct::core::DerivativesCppadSettings settings;
	settings.createSparseJacobian_ = true;

	Eigen::VectorXi jac_rows, jac_cols;
	constraint_Codegen_->compileJIT(settings, "IntermediateGeneralPositionConstraints_Vehicle");
	constraint_Codegen_->getSparsityPatternJacobian(jac_rows, jac_cols);

	size_t stateIndex = 0;
	size_t inputIndex = 0;

	int nonZerosState = (jac_cols.array() < STATE_DIM).count();
	int nonZerosInput = (jac_cols.array() >= STATE_DIM).count();

	jac_rows_state.resize(nonZerosState);
	jac_cols_state.resize(nonZerosState);
	jac_rows_input.resize(nonZerosInput);
	jac_cols_input.resize(nonZerosInput);

	for (int i = 0; i < jac_rows.rows(); ++i)
    {
        if (jac_cols(i) < static_cast<int>(STATE_DIM))
        {
            jac_rows_state(stateIndex) = jac_rows(i);
            jac_cols_state(stateIndex) = jac_cols(i);
            stateIndex++;
        }
        else
        {
            jac_rows_input(inputIndex) = jac_rows(i);
            jac_cols_input(inputIndex) = jac_cols(i) - STATE_DIM;
            inputIndex++;
        }
    }

    initialized = true;
}