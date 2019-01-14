#include <ct/core/core.h>
#include <Eigen/Dense>
#include <iostream>

#include "Vehicle.h"
#include "IntermediateGeneralConstraintsVehicle.h"
#include "TestTrack.h"
#include "get_AccelerationConstraints.h"
#include "IntermediateGeneralPositionConstraint.h"
#include "TerminalLinearTerm.h"

static const size_t state_dim = 6;
static const size_t control_dim = 2;
static const std::shared_ptr<TestTrack> test_track(new TestTrack());
static const std::shared_ptr<AccelerationLimits> acceleration_limits(new AccelerationLimits());


int main(int argc, char const *argv[])
{	
	typedef CppAD::AD< CppAD::cg::CG< double > > ADCGScalar;
	
	std::shared_ptr< ct::core::DiscreteControlledSystem<state_dim, control_dim, ADCGScalar> > vehicle_dynamic(
		new ct::core::Vehicle<ADCGScalar>(0.1));

	std::shared_ptr< ct::core::DiscreteSystemLinearizerADCG<state_dim, control_dim> > linearizerADCG(
		new ct::core::DiscreteSystemLinearizerADCG<state_dim, control_dim>(vehicle_dynamic));



	Eigen::Matrix<double, 6, 1> x;
	x << 1, 1, 100, 200, 3, 4;

	Eigen::Matrix<double, 2, 1> u;
	u << -100, 50;

	linearizerADCG->compileJIT();
	auto B = linearizerADCG->getDerivativeControl(x, u);
	auto A = linearizerADCG->getDerivativeState(x, u);
	std::cout << B << std::endl << std::endl;
	std::cout << A << std::endl;

	std::shared_ptr< ct::optcon::ConstraintContainerAD<state_dim, control_dim> > general_constraint(
		new ct::optcon::ConstraintContainerAD<state_dim, control_dim>());

	std::shared_ptr<ct::core::StateVectorArray<state_dim, double>> previous_optimal_states(
		new ct::core::StateVectorArray<state_dim, double>());

	previous_optimal_states->resize(50);

	double i(0.);
	for (size_t  k = 0; k < 50; k++)
	{
		ct::core::StateVector<state_dim, double> x_temp;
		x_temp << 6.1627, 0.0434, 10.652, 0.3505, 0, 0;
		(*previous_optimal_states)[k] = x_temp;
		i += 1.;
	}

	ct::core::ControlVector<control_dim, double> u0;
	u0 << 0.,0.;
	ct::core::StateVector<state_dim, double> x0;
	x0 << 1.,1.,1.,1.,1.,1.;
	

	std::shared_ptr< ct::optcon::ConstraintBase<state_dim, control_dim, double> >intermediate_general_constraint(
		new ct::optcon::IntermediateGeneralConstraintsVehicle<state_dim, control_dim, double>(previous_optimal_states, acceleration_limits, 0.01));

	intermediate_general_constraint->getNumNonZerosJacobianState();
	intermediate_general_constraint->getNumNonZerosJacobianInput();

	auto jac = intermediate_general_constraint->jacobianState(x0, u0, 0.01);
	auto sparse = intermediate_general_constraint->jacobianStateSparse(x0, u0, 0.01);
	std::cout << jac<< std::endl;
	std::cout << sparse.transpose() << std::endl;

	jac = intermediate_general_constraint->jacobianInput(x0, u0, 0.01);
	sparse = intermediate_general_constraint->jacobianInputSparse(x0, u0, 0.01);
	std::cout << jac<< std::endl;
	std::cout << sparse.transpose() << std::endl;

	Eigen::Matrix<double, 2, 1> left_;
	Eigen::Matrix<double, 2, 1> right_;
	Eigen::Matrix<double, 2, 1> forward_vector_;
	double p_x, p_y;

	test_track->get_Track(left_, right_, forward_vector_, p_x, p_y, (*previous_optimal_states)[0]);
	auto idx = test_track->get_CheckPointIndex((*previous_optimal_states)[0]);
	std::cout << left_ << std::endl << right_ << std::endl << forward_vector_ << std::endl;
	std::cout << idx << std::endl;

	std::cout << std::endl;
	std::shared_ptr< ct::optcon::ConstraintBase<state_dim, control_dim, double> >intermediate_general_position_constraint(
		new ct::optcon::IntermediateGeneralPositionConstraint<state_dim, control_dim, double>(previous_optimal_states, test_track, 0.01));

	intermediate_general_position_constraint->getNumNonZerosJacobianState();
	intermediate_general_position_constraint->getNumNonZerosJacobianInput();

	jac = intermediate_general_position_constraint->jacobianState(x0, u0, 0.01);
	sparse = intermediate_general_position_constraint->jacobianStateSparse(x0, u0, 0.01);
	std::cout << jac<< std::endl;
	std::cout << sparse.transpose() << std::endl;

	jac = intermediate_general_position_constraint->jacobianInput(x0, u0, 0.01);
	sparse = intermediate_general_position_constraint->jacobianInputSparse(x0, u0, 0.01);
	std::cout << jac<< std::endl;
	std::cout << sparse.transpose() << std::endl;

	std::shared_ptr<ct::optcon::TerminalLinearTerm<state_dim, control_dim>> terminal_term_vehicle(
		new ct::optcon::TerminalLinearTerm<state_dim, control_dim>(test_track, previous_optimal_states));
	std::cout << std::endl << terminal_term_vehicle->stateDerivative(x0, u0, 0.01) << std::endl;
	std::cout << terminal_term_vehicle->stateSecondDerivative(x0, u0, 0.01) << std::endl;
	std::cout << terminal_term_vehicle->controlDerivative(x0,u0,0.01) << std::endl;
	std::cout << terminal_term_vehicle->controlSecondDerivative(x0, u0, 0.01) << std::endl;
	std::cout << terminal_term_vehicle->stateControlDerivative(x0, u0, 0.01) << std::endl;
	return 0;
}