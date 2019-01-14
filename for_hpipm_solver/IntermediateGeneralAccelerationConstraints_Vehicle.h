#pragma once

#include <ct/optcon/optcon.h>
#include <Eigen/Dense>
#include "get_AccelerationConstraints.h"


namespace ct{
namespace optcon{

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class IntermediateGeneralAccelerationConstraints_Vehicle : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Base = ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>;
	using state_vector_t = typename Base::state_vector_t;
	using control_vector_t = typename Base::control_vector_t;
	using VectorXs = typename Base::VectorXs;
	using MatrixXs = typename Base::MatrixXs;

	typedef core::DerivativesCppadJIT<STATE_DIM + CONTROL_DIM, -1> JacCG;
	typedef typename JacCG::CG_SCALAR CGScalar;

	IntermediateGeneralAccelerationConstraints_Vehicle(
		std::shared_ptr< ct::core::StateVectorArray<STATE_DIM, SCALAR> > previous_optimal_states,
		std::shared_ptr<AccelerationLimits<STATE_DIM, CONTROL_DIM>> accelerationlimits,
		const double& delta_t);

	IntermediateGeneralAccelerationConstraints_Vehicle(const IntermediateGeneralAccelerationConstraints_Vehicle& arg);

	virtual IntermediateGeneralAccelerationConstraints_Vehicle<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
	{
		return (new IntermediateGeneralAccelerationConstraints_Vehicle(*this)); 
	}

	virtual ~IntermediateGeneralAccelerationConstraints_Vehicle() = default;

	virtual VectorXs evaluate(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

	virtual size_t getConstraintSize() const override;

	virtual VectorXs getLowerBound() const override;
	virtual VectorXs getUpperBound() const override;

	virtual MatrixXs jacobianState(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;
	virtual MatrixXs jacobianInput(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

	virtual size_t getNumNonZerosJacobianState() const override;
	virtual size_t getNumNonZerosJacobianInput() const override;

	virtual VectorXs jacobianStateSparse(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;
	virtual VectorXs jacobianInputSparse(const state_vector_t& x, const control_vector_t& u, const SCALAR t) override;

	virtual void sparsityPatternState(Eigen::VectorXi& rows, Eigen::VectorXi& cols) override;
	virtual void sparsityPatternInput(Eigen::VectorXi& rows, Eigen::VectorXi& cols) override;

private:
	std::shared_ptr< ct::core::StateVectorArray<STATE_DIM, SCALAR> > previous_optimal_states_;
	std::shared_ptr<AccelerationLimits<STATE_DIM, CONTROL_DIM>> accelerationlimits_;

	const double delta_t_;

	Eigen::VectorXi jac_rows_state;
	Eigen::VectorXi jac_cols_state;
	Eigen::VectorXi jac_rows_input;
	Eigen::VectorXi jac_cols_input;


	Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> CppadCg(
		const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
		const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
		ct::core::ADCGScalar t);

	void initialize();
};

#include "IntermediateGeneralAccelerationConstraints_Vehicle-impl.cpp"
}
}