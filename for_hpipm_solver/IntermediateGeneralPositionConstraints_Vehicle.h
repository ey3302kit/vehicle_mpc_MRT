#pragma once

#include <ct/optcon/optcon.h>
#include <Eigen/Dense>
#include "TestTrack.h"


namespace ct{
namespace optcon{

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class IntermediateGeneralPositionConstraints_Vehicle : public ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	using Base = ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>;
	using state_vector_t = typename Base::state_vector_t;
	using control_vector_t = typename Base::control_vector_t;
	using VectorXs = typename Base::VectorXs;
	using MatrixXs = typename Base::MatrixXs;

	using JacCG = core::DerivativesCppadJIT<STATE_DIM + CONTROL_DIM, -1>;
	using CGScalar = typename JacCG::CG_SCALAR;

	IntermediateGeneralPositionConstraints_Vehicle(
		std::shared_ptr< ct::core::StateVectorArray<STATE_DIM, SCALAR> > previous_optimal_states,
		std::shared_ptr<TestTrack<STATE_DIM, CONTROL_DIM>> test_track,
		const double& delta_t);

	IntermediateGeneralPositionConstraints_Vehicle(const IntermediateGeneralPositionConstraints_Vehicle& arg);

	virtual IntermediateGeneralPositionConstraints_Vehicle* clone() const override
	{ return new IntermediateGeneralPositionConstraints_Vehicle(*this); }

	virtual ~IntermediateGeneralPositionConstraints_Vehicle() = default;

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
	std::shared_ptr<TestTrack<STATE_DIM, CONTROL_DIM>> test_track_;

	const double delta_t_;

	Eigen::VectorXi jac_rows_state;
	Eigen::VectorXi jac_cols_state;
	Eigen::VectorXi jac_rows_input;
	Eigen::VectorXi jac_cols_input;

	bool initialized = false;

	double L = 3.;

	Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> CppadCg(
		const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
		const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
		ct::core::ADCGScalar t);

	void initialize();
};

#include "IntermediateGeneralPositionConstraints_Vehicle-impl.cpp"

}
}