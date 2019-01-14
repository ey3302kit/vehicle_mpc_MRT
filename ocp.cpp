#include <ct/core/core.h>
#include <iostream>

#include "Vehicle.h"

static const size_t STATE_DIM = 6;
static const size_t CONTROL_DIM = 2;
static const bool DISCRETE = true;
static const size_t P_DIM = 2;
static const size_t V_DIM = 2;

int main(int argc, char const *argv[])
{
	typedef CppAD::AD< CppAD::cg::CG<double> > ADCGScalar;
	typedef 

	// sampling 
	const double delta_t = 0.1;

	// setup the system and its linearization
	std::shared_ptr< ct::core::DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, ADCGScalar> > system_dynamic(
		new ct::core::Vehicle<ADCGScalar>(delta_t));

	std::shared_ptr< ct::core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, ADCGScalar> > linearizer(
		new ct::core::DiscreteSystemLinearizerADCG<STATE_DIM, CONTROL_DIM>(system_dynamic)); 
	//end

	// setup the constraints

	// end

	// setup the objective function
	std::shared_ptr< ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM> > costfunction(
		new ct::optcon::CostFunctionAD<STATE_DIM, CONTROL_DIM>());

	// intermediate term
	Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q(Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero());
	Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> R;
	R << .1, 0, 0, .1;

	std::shared_ptr< ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM> > intermediate_term(
		new ct::optcon::TermQuadratic<STATE_DIM, CONTROL_DIM>(Q, R));

	// final term
	// final_term
	costfunction->addIntermediateADTerm(intermediate_term);
	costfunction->addFinaADTerm(final_term);
	// end

	// setup OCP
	ct::optcon::DiscreteOptConProblem<STATE_DIM, CONTROL_DIM> ocp(
		system_dynamic, costfunction, linearizer);
	// end

	// setup settings
	ct::optcon::NLOptConSettings ocp_settings;
	// end

	// setup a NLOptConSolver
	NLOptConSolver<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, double, DISCRETE> ocp_solver(ocp, ocp_settings);

	/* NLOptConSolver -> NLOCBackendST -> NLOCBackendBase -> SystemInterface -> NLOCBackendBase -> SystemInterface::initialize()
		-> NLOCBackendBase::configure() -> SystemInterface::configure() -> HPIPMInterface new -> HPIPMInterface::configure() (solver) -> NLOCBackendBase::NLOCBackendBase()
		-> changeCostFunction() -> NLOptConSolver::initialize() ->NLOCAlgorithm::GNMS(NLOCBackendBase, settings)
	*/
	// end

	ocp_solver.solve();

	/*	
	NLOptConSolver::solve -> NLOptConSolver::runIteration -> NLOCAlgorithm::runIteration() -> GNMS::runIteration() -> GNMS::prepareIteration() -> 
	NLOCBackendBase::getNumSteps() -> NLOCAlgorithm::getNumStepsPerShot() -> NLOCBackendST::rolloutShots() -> NLOCBackendBase::rolloutShotsSingleThreaded() ->
	
	*/

	return 0;
}