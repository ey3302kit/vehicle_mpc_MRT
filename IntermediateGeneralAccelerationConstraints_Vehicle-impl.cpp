template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::IntermediateGeneralAccelerationConstraints_Vehicle(
	std::shared_ptr< ct::core::StateVectorArray<STATE_DIM, SCALAR> > previous_optimal_states,
	std::shared_ptr<AccelerationLimits<STATE_DIM, CONTROL_DIM>> accelerationlimits,
	const double& delta_t) : 
	previous_optimal_states_(previous_optimal_states),
	accelerationlimits_(accelerationlimits),
	delta_t_(delta_t)
{
	this->lb_.resize(getConstraintSize());
	this->ub_.resize(getConstraintSize());

	this->ub_.setConstant(SCALAR(0.0));
	this->lb_.setConstant(-1e5);
	//this->lb_.setConstant(-std::numeric_limits<SCALAR>::infinity());
	initialize();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::IntermediateGeneralAccelerationConstraints_Vehicle(
	const IntermediateGeneralAccelerationConstraints_Vehicle& arg) : 
	previous_optimal_states_(arg.previous_optimal_states_),
	accelerationlimits_(arg.accelerationlimits_),
	delta_t_(arg.delta_t_),
	jac_rows_state(arg.jac_rows_state),
	jac_cols_state(arg.jac_cols_state),
	jac_rows_input(arg.jac_rows_input),
	jac_cols_input(arg.jac_cols_input)
{
	this->lb_ = arg.lb_;
	this->ub_ = arg.ub_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM ,typename SCALAR>
typename IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs 
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{	
	VectorXs val;

	size_t i = size_t(t / delta_t_);

	std::vector< Eigen::Matrix<double, 1, 2> > A;
	std::vector< double > b;
	accelerationlimits_->get_AccelerationConstraints(A, b, (*previous_optimal_states_)[i]);

	val.resize(A.size());

	for (size_t j = 0; j < A.size(); j++)
		val(j) = (A[j] * x.segment(4, 2))(0) - b[j];

	return val;

}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> 
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::CppadCg(
	const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
	const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
	ct::core::ADCGScalar t)
{
	Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> val(getConstraintSize());

	size_t i = CppAD::Value(t).getValue() / delta_t_;

	// acceleration constraints
	std::vector< Eigen::Matrix<double, 1, 2> > A(getConstraintSize());
	std::vector<double> b(getConstraintSize());
	//accelerationlimits_->get_AccelerationConstraints(A, b, (*previous_optimal_states_)[i]);


	for (size_t j = 0; j < A.size(); j++)
	{
		A[j] << 1., 1.;
		b[j] = 1.;
		val(j) = ((A[j].template cast<ct::core::ADCGScalar>()) * x.segment(4, 2))(0) - ct::core::ADCGScalar(b[j]);
	}

	return val;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getConstraintSize() const
{
	return 16;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs 
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getLowerBound() const
{
	return this->lb_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs 
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getUpperBound() const
{
	return this->ub_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::jacobianState(
	const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	MatrixXs jac(getConstraintSize(), STATE_DIM);
	jac.setZero();

	VectorXs sparse(jacobianStateSparse(x, u, t));

	for (size_t i = 0; i < getNumNonZerosJacobianState(); i++)
		jac(jac_rows_state(i), jac_cols_state(i)) = sparse(i);

	return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::jacobianInput(
	const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	MatrixXs jac(getConstraintSize(), CONTROL_DIM);
	jac.setZero();

	return jac;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianState() const
{
	return (jac_rows_state.size());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::getNumNonZerosJacobianInput() const
{
	return (jac_rows_input.size());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::jacobianStateSparse(
	const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	VectorXs sparse(getNumNonZerosJacobianState());

	size_t i = size_t(t / delta_t_);

	std::vector< Eigen::Matrix<double, 1, 2> > A;
	std::vector<double> b;
	accelerationlimits_->get_AccelerationConstraints(A, b, (*previous_optimal_states_)[i]);

	for (size_t j = 0; j < getConstraintSize(); j ++)
	{
		for (size_t k = 0; k < 2; k++)
		{
			sparse(j * 2 + k) = A[j](k);
		}
	}

	return sparse;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::jacobianInputSparse(
	const state_vector_t& x, const control_vector_t& u, const SCALAR t)
{
	VectorXs sparse(getNumNonZerosJacobianInput());
	return sparse;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::sparsityPatternState(
	Eigen::VectorXi& rows, Eigen::VectorXi& cols)
{
	rows.resize(jac_rows_state.size());
	cols.resize(jac_cols_state.size());

	rows = jac_rows_state;
	cols = jac_cols_state;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::sparsityPatternInput(
	Eigen::VectorXi& rows, Eigen::VectorXi& cols)
{
	rows.resize(jac_rows_input.size());
	cols.resize(jac_cols_input.size());

	rows = jac_rows_input;
	cols = jac_cols_input;
}
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM ,CONTROL_DIM ,SCALAR>::initialize()
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
	constraint_Codegen_->compileJIT(settings, "IntermediateGeneralAccelerationConstraints_Vehicle");
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
}