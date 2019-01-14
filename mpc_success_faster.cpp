#include <ct/core/core.h>
#include <Eigen/Dense>

#include <iostream>

#include "Vehicle2.h"
#include "IntermediateGeneralConstraintsVehicle.h"
#include "TestTrack.h"
#include "get_AccelerationConstraints.h"
#include "IntermediateGeneralPositionConstraint.h"
#include "TerminalLinearTerm.h"
#include "TerminalConstraintVehicle.h"

static const size_t state_dim = 6;
static const size_t control_dim = 3;
static const std::shared_ptr<TestTrack<state_dim, control_dim>> test_track(new TestTrack<state_dim, control_dim>());
static const std::shared_ptr<AccelerationLimits<state_dim, control_dim>> acceleration_limits(new AccelerationLimits<state_dim, control_dim>());


int main(int argc, char const *argv[])
{
	typedef CppAD::AD< CppAD::cg::CG< double > > ADCGScalar;

	std::shared_ptr<ct::core::StateVectorArray<state_dim, double>> previous_optimal_states(
		new ct::core::StateVectorArray<state_dim, double>());

	previous_optimal_states->resize(100);

	double i(0.);

	for (size_t  k = 0; k < 100; k++)
	{
		ct::core::StateVector<state_dim, double> x_temp;
		x_temp << 0,0, 0, 0, 0, 0;
		(*previous_optimal_states)[k] = x_temp;
		i += 1.;
	}
    
        // (*previous_optimal_states)[0] << 0.910235103935618,   0.000984788796091803 ,   2.72383162713817 ,  -0.0323150517433697,0,0;
        // (*previous_optimal_states)[1] <<1.21074686461553 ,   -0.00508322966797777 ,   3.28640358646007  ,  -0.0890453175379540,0,0;
        // (*previous_optimal_states)[2] <<1.57104108278073  ,  -0.0167219955841523, 3.91948077684412  ,  -0.143730000785468,0,0;
        // (*previous_optimal_states)[3] <<1.99849340836560  ,  -0.0336968751477200, 4.62956573485335 ,   -0.195767590485819,0,0;
        // (*previous_optimal_states)[4] <<2.50135072041140  ,  -0.0557060668540799 ,5.42758050606258 ,   -0.244416243641311,0,0;
        // (*previous_optimal_states)[5] <<3.08889770361311   , -0.0823757735923512, 6.32335915797183 ,   -0.288977891124047,0,0;
        // (*previous_optimal_states)[6] <<3.77037193239607  ,  -0.113292717459732 , 7.30612541768737 ,   -0.329360986223510,0,0;
        // (*previous_optimal_states)[7] <<4.55345596305186  ,  -0.148048824041436 , 8.35555519542745 ,   -0.365761145410492,0,0;
        // (*previous_optimal_states)[8] <<5.44385310674106  ,  -0.186255262591568 , 9.45238767835609 ,   -0.398367625592072,0,0;
        // (*previous_optimal_states)[9] <<6.44539398439220  ,  -0.227541811406883 , 10.5784298746660 ,   -0.427363350714176,0,0;
        // (*previous_optimal_states)[10] <<7.56014334457088  ,  -0.271556226072108 , 11.7165573289070 ,   -0.452924942590257,0,0;
        // (*previous_optimal_states)[11] <<8.78850692520502  ,  -0.317963609685109 , 12.8507142837750 ,   -0.475222729669692,0,0;
        // (*previous_optimal_states)[12] <<10.1293383235735  ,  -0.366445783598491 , 13.9659136835936 ,   -0.494420748597870,0,0;
        // (*previous_optimal_states)[13] <<11.5800458661274  ,  -0.416700658327503 , 15.0482371674833 ,   -0.510676745982310,0,0;
        // (*previous_optimal_states)[14] <<13.1366994781557  ,  -0.468441604624404 , 16.0848350730817 ,   -0.524142179955649,0,0;
        // (*previous_optimal_states)[15] <<14.7941375546056  ,  -0.521396824699683 , 17.0639264559155 ,   -0.534962221549847,0,0;
        // (*previous_optimal_states)[16] <<16.5460738330551  ,  -0.575308723578110 , 17.9747991130735 ,   -0.543275756018634,0,0;
        // (*previous_optimal_states)[17] <<18.3852042688213  ,  -0.629933280589029 , 18.8078096022492 ,   -0.549215384199686,0,0;
        // (*previous_optimal_states)[18] <<20.3033139116311  ,  -0.685039420996724 , 19.5543832539458 ,   -0.552907423954127,0,0;
        // (*previous_optimal_states)[19] <<22.2913837832426  ,  -0.740408387780313 , 20.2070141782826 ,   -0.554471911717584,0,0;
        // (*previous_optimal_states)[20] <<24.3396977555775  ,  -0.795833113576091 , 20.7592652684150 ,   -0.554022604197913,0,0;
        // (*previous_optimal_states)[21] <<26.4379494291471  ,  -0.851117592796144 , 21.2057682029758 ,   -0.551666980203082,0,0;
        // (*previous_optimal_states)[22] <<28.5753490116592  ,  -0.906076253942335 , 21.5422234472647 ,   -0.547506242720664,0,0;
        // (*previous_optimal_states)[23] <<30.7407301967187  ,  -0.960533332141261 , 21.7654002539229 ,   -0.541635321257795,0,0;
        // (*previous_optimal_states)[24] <<32.9226570425977  ,  -1.01432224192787  , 21.8731366636556 ,   -0.534142874474336,0,0;
        // (*previous_optimal_states)[25] <<35.1095308510839  ,  -1.06728495030980  , 21.8643395060674 ,   -0.525111293164165,0,0;
        // (*previous_optimal_states)[26] <<37.2896970464098  ,  -1.11927135015036  , 21.7389844004501 ,   -0.514616703646888,0,0;
        // (*previous_optimal_states)[27] <<39.4515520542383  ,  -1.17013863391486  , 21.4981157561189 ,   -0.502728971643066,0,0;
        // (*previous_optimal_states)[28] <<41.5836501806168  ,  -1.21975066783314  , 21.1438467714491 ,   -0.489511706722553,0,0;
        // (*previous_optimal_states)[29] <<43.6748104906976  ,  -1.26797736654124  , 20.6793594301661 ,   -0.475022267439449,0,0;
        // (*previous_optimal_states)[30] <<45.7142236868568  ,  -1.31469406827659  , 20.1089044930162 ,   -0.459311767267482,0,0;
        // (*previous_optimal_states)[31] <<47.6915589856396  ,  -1.35978091071284  , 19.4378014826392 ,   -0.442425081457368,0,0;
        // (*previous_optimal_states)[32] <<49.5970709927969  ,  -1.40312220753285  , 18.6724386605052 ,   -0.424400854942760,0,0;
        // (*previous_optimal_states)[33] <<51.4217065757808  ,  -1.44460582584521  , 17.8202729991709 ,   -0.405271511304312,0,0;
        // (*previous_optimal_states)[34] <<53.1572117332771  ,  -1.48412256455449  , 16.8898301507538 ,   -0.385063262881209,0,0;
        // (*previous_optimal_states)[35] <<54.7962384603141  ,  -1.52156553381799  , 15.8907043899855 ,   -0.363796122388789,0,0;
        // (*previous_optimal_states)[36] <<56.3324516027290  ,  -1.55682953576422  , 14.8335584583116 ,   -0.341483916535777,0,0;
        // (*previous_optimal_states)[37] <<57.7606356830400  ,  -1.58981044668817  , 13.7301231479094 ,   -0.318134301943182,0,0;
        // (*previous_optimal_states)[38] <<59.0768016454738  ,  -1.62040460113305  , 12.5931961007642 ,   -0.293748786954372,0,0;
        // (*previous_optimal_states)[39] <<60.2782933281280  ,  -1.64850818018642  , 11.4366375523205 ,   -0.268322794112897,0,0;
        // (*previous_optimal_states)[40] <<61.3638923696304  ,  -1.67401662889248  , 10.2753432777269 ,   -0.241846180008300,0,0;
        // (*previous_optimal_states)[41] <<62.3337357928043  ,  -1.69682805992818  , 9.12152518574963 ,   -0.214382440705639,0,0;
        // (*previous_optimal_states)[42] <<63.1885740732290  ,  -1.71686106600008  , 7.97524042274403 ,   -0.186277680732161,0,0;
        // (*previous_optimal_states)[43] <<63.9291600729384  ,  -1.73407011849311  , 6.83647957144411 ,   -0.157903369128332,0,0;
        // (*previous_optimal_states)[44] <<64.5562412272842  ,  -1.74844812043494  , 5.70514351547083 ,   -0.129656669708345,0,0;
        // (*previous_optimal_states)[45] <<65.0705551709447  ,  -1.76002883956556  , 4.58113535773938 ,   -0.101957712903904,0,0;
        // (*previous_optimal_states)[46] <<65.4728101324508  ,  -1.76888955896150  , 3.46396387238178 ,   -0.0752566750148948,0,0;
        // (*previous_optimal_states)[47] <<65.7635737836824  ,  -1.77515539987763  , 2.35130915224976 ,   -0.0500601433075103,0,0;
        // (*previous_optimal_states)[48] <<65.9405626577026  ,  -1.77890819097671  , 1.18846832815615 ,   -0.0249956786741983,0,0;
        // (*previous_optimal_states)[49] <<65.9999860741105  ,  -1.78015797491043  , 0   ,0,0,0;
	//
	std::shared_ptr< ct::core::DiscreteControlledSystem<state_dim, control_dim, double> > vehicle_dynamic(
		new ct::core::Vehicle<double>(0.1));

	//std::shared_ptr< ct::core::DiscreteSystemLinearizerADCG<state_dim, control_dim> > linearizerADCG(
	//	new ct::core::DiscreteSystemLinearizerADCG<state_dim, control_dim>(vehicle_dynamic));
	std::shared_ptr< ct::core::DiscreteSystemLinearizer<state_dim, control_dim> > linearizerADCG(
		new ct::core::DiscreteSystemLinearizer<state_dim, control_dim>(vehicle_dynamic));

	// cost function
    std::shared_ptr<ct::core::tpl::SingleActivation<double>> activation(
        new ct::core::tpl::SingleActivation<double>(0.05, 1000));
    std::shared_ptr<ct::core::tpl::SingleActivation<double>> activation_0(
        new ct::core::tpl::SingleActivation<double>(-1, 0.05));
	Eigen::Matrix<double, state_dim, state_dim> Q(Eigen::Matrix<double, state_dim, state_dim>::Zero());
	Eigen::Matrix<double, control_dim, control_dim> R(Eigen::Matrix<double, control_dim, control_dim>::Zero());
	R(0, 0) = 0.25;
	R(1, 1) = 0.25;
    R(2, 2) = 1e8;
    Eigen::Matrix<double, control_dim, control_dim> R_0(Eigen::Matrix<double, control_dim, control_dim>::Zero());
    R_0(2, 2) = 1e8;


	std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediate_cost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>(Q, R));
    intermediate_cost->setTimeActivation(activation, true);
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediate_cost_0(
        new ct::optcon::TermQuadratic<state_dim, control_dim>(Q, R_0));
    intermediate_cost_0->setTimeActivation(activation_0, true);
    std::shared_ptr<ct::optcon::TerminalLinearTerm<state_dim, control_dim>> final_cost(
        new ct::optcon::TerminalLinearTerm<state_dim, control_dim>(test_track, previous_optimal_states));

    std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());
    costFunction->addIntermediateTerm(intermediate_cost);
    costFunction->addIntermediateTerm(intermediate_cost_0);
    costFunction->addFinalTerm(final_cost);

    // constraint
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> generalConstraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> boxConstraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>> intermediate_general_acceleration_constraint(
		new ct::optcon::IntermediateGeneralConstraintsVehicle<state_dim, control_dim, double>(previous_optimal_states, acceleration_limits, 0.1));

    Eigen::Matrix<double, 2, 1> x_u_b;
    Eigen::Matrix<double, 2, 1> x_l_b;
    x_u_b << 40, 40;
    x_l_b << -40, -40;
    Eigen::Matrix<int, state_dim, 1> sparse;
    sparse << 0, 0, 0, 0, 1, 1;
    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>> intermediate_box_state_constraint(
        new ct::optcon::StateConstraint<state_dim, control_dim, double>(x_l_b, x_u_b, sparse));
    Eigen::Matrix<double, 3, 1> u_u_b;
    Eigen::Matrix<double, 3, 1> u_l_b;
    u_u_b << 40, 40, 1e5;
    u_l_b << -40, -40, 0;
    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>> intermediate_box_control_constraint(
        new ct::optcon::ControlInputConstraint<state_dim, control_dim, double>(u_l_b, u_u_b));
    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>> intermediate_general_position_constraint(
		new ct::optcon::IntermediateGeneralPositionConstraint<state_dim, control_dim, double>(previous_optimal_states, test_track, 0.1));
    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>>terminal_constraint_vehicle(
        new ct::optcon::TerminalConstraintVehicle<state_dim, control_dim, double>());


    generalConstraints->addIntermediateConstraint(intermediate_general_acceleration_constraint, true);
    generalConstraints->addIntermediateConstraint(intermediate_general_position_constraint, true);

    boxConstraints->addIntermediateConstraint(intermediate_box_control_constraint, true);
    boxConstraints->addIntermediateConstraint(intermediate_box_state_constraint, true);
    boxConstraints->addTerminalConstraint(terminal_constraint_vehicle, true);
    //boxConstraints->addTerminalConstraint(intermediate_box_control_constraint, true);
    boxConstraints->addTerminalConstraint(intermediate_box_state_constraint, true);


    generalConstraints->addTerminalConstraint(intermediate_general_acceleration_constraint, true);
    generalConstraints->addTerminalConstraint(intermediate_general_position_constraint, true);
    generalConstraints->initialize();

    // ocp
    size_t horizon = 50;
    ct::core::StateVector<state_dim, double> x0;
    x0 << 0,    0 ,   0  ,  0,0,0;

    ct::optcon::DiscreteOptConProblem<state_dim, control_dim> ocp(
    	horizon, x0, vehicle_dynamic, costFunction, linearizerADCG);

    ocp.setGeneralConstraints(generalConstraints);
    ocp.setBoxConstraints(boxConstraints);

    ct::optcon::NLOptConSettings nloc_settings;
    nloc_settings.dt = 0.1;  // the control discretization in [sec]
    nloc_settings.integrator = ct::core::IntegrationType::EULERCT;
    nloc_settings.discretization = ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    nloc_settings.max_iterations = 20;
    nloc_settings.min_cost_improvement = 1e-3;
    nloc_settings.nThreads = 1;
    nloc_settings.nlocp_algorithm = ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::GNMS;
    nloc_settings.lqocp_solver = ct::optcon::NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // solve LQ-problems using HPIPM
    nloc_settings.lqoc_solver_settings.num_lqoc_iterations = 50;                // number of riccati sub-iterations
    
    nloc_settings.maxDefectSum = (1e-5);
    nloc_settings.lqoc_solver_settings.lqoc_debug_print = true;

    nloc_settings.lineSearchSettings.active = true;
    nloc_settings.lineSearchSettings.debugPrint = true;
    nloc_settings.printSummary = true;

    ct::core::FeedbackArray<state_dim, control_dim> u0_fb(horizon , ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());
    ct::core::ControlVectorArray<control_dim> u0_ff(horizon, ct::core::ControlVector<control_dim>::Zero());
    u0_ff.resize(horizon);
    ct::core::ControlVector<control_dim, double> u0_guess;
    u0_guess << 0, 0, 0;
    u0_ff.setConstant(u0_guess);
    ct::core::StateVectorArray<state_dim> x_ref_init(horizon + 1, ct::core::StateVector<state_dim>::Zero());

    ct::optcon::NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);
    ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc(ocp, nloc_settings);
    ct::core::StateFeedbackController<state_dim, control_dim> solution;

    //auto backend = nloc.getBackend();
    //std::cout << backend->lqocSolver_->getSolutionState() << std::endl;
    //std::cout << backend->lqocSolver_->getSolutionControl() << std::endl;
    // nloc_0.setInitialGuess(initController);
    // nloc_0.solve();
    // solution = nloc_0.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_0.setInitialGuess(initController);
    // nloc_0.solve();
    // solution = nloc_0.getSolution();

    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_1(ocp, nloc_settings);
    // nloc_1.setInitialGuess(initController);
    // nloc_1.solve();
    // solution = nloc_1.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_1.setInitialGuess(initController);
    // nloc_1.solve();
    // solution = nloc_1.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_2(ocp, nloc_settings);
    // nloc_2.setInitialGuess(initController);
    // nloc_2.solve();
    // solution = nloc_2.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_2.setInitialGuess(initController);
    // nloc_2.solve();
    // solution = nloc_2.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_3(ocp, nloc_settings);
    // nloc_3.setInitialGuess(initController);
    // nloc_3.solve();
    // solution = nloc_3.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_3.setInitialGuess(initController);
    // nloc_3.solve();
    // solution = nloc_3.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_4(ocp, nloc_settings);
    // nloc_4.setInitialGuess(initController);
    // nloc_4.solve();
    // solution = nloc_4.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_4.setInitialGuess(initController);
    // nloc_4.solve();
    // solution = nloc_4.getSolution();

    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_5(ocp, nloc_settings);
    // nloc_5.setInitialGuess(initController);
    // nloc_5.solve();
    // solution = nloc_5.getSolution();
    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_5.setInitialGuess(initController);
    // nloc_5.solve();
    // solution = nloc_5.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_6(ocp, nloc_settings);
    // nloc_6.setInitialGuess(initController);
    // nloc_6.solve();
    // solution = nloc_6.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_6.setInitialGuess(initController);
    // nloc_6.solve();
    // solution = nloc_6.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_7(ocp, nloc_settings);
    // nloc_7.setInitialGuess(initController);
    // nloc_7.solve();
    // solution = nloc_7.getSolution();
    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_7.setInitialGuess(initController);
    // nloc_7.solve();
    // solution = nloc_7.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_8(ocp, nloc_settings);
    // nloc_8.setInitialGuess(initController);
    // nloc_8.solve();
    // solution = nloc_8.getSolution();
    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_8.setInitialGuess(initController);
    // nloc_8.solve();
    // solution = nloc_8.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_9(ocp, nloc_settings);
    // nloc_9.setInitialGuess(initController);
    // nloc_9.solve();
    // solution = nloc_9.getSolution();
    //     *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_9.setInitialGuess(initController);
    // nloc_9.solve();
    // solution = nloc_9.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_10(ocp, nloc_settings);
    // nloc_10.setInitialGuess(initController);
    // nloc_10.solve();
    // solution = nloc_10.getSolution();
    //     *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_10.setInitialGuess(initController);
    // nloc_10.solve();
    // solution = nloc_10.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_11(ocp, nloc_settings);
    // nloc_11.setInitialGuess(initController);
    // nloc_11.solve();
    // solution = nloc_11.getSolution();
    //     *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_11.setInitialGuess(initController);
    // nloc_11.solve();
    // solution = nloc_11.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_12(ocp, nloc_settings);
    // nloc_12.setInitialGuess(initController);
    // nloc_12.solve();
    // solution = nloc_12.getSolution();
    //     *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_12.setInitialGuess(initController);
    // nloc_12.solve();
    // solution = nloc_12.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_13(ocp, nloc_settings);
    // nloc_13.setInitialGuess(initController);
    // nloc_13.solve();
    // solution = nloc_13.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_13.setInitialGuess(initController);
    // nloc_13.solve();
    // solution = nloc_13.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_14(ocp, nloc_settings);
    // nloc_14.setInitialGuess(initController);
    // nloc_14.solve();
    // solution = nloc_14.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_14.setInitialGuess(initController);
    // nloc_14.solve();
    // solution = nloc_14.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_15(ocp, nloc_settings);
    // nloc_15.setInitialGuess(initController);
    // nloc_15.solve();
    // solution = nloc_15.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_15.setInitialGuess(initController);
    // nloc_15.solve();
    // solution = nloc_15.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_16(ocp, nloc_settings);
    // nloc_16.setInitialGuess(initController);
    // nloc_16.solve();
    // solution = nloc_16.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_16.setInitialGuess(initController);
    // nloc_16.solve();
    // solution = nloc_16.getSolution();

    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_17(ocp, nloc_settings);
    // nloc_17.setInitialGuess(initController);
    // nloc_17.solve();
    // solution = nloc_17.getSolution();
    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_17.setInitialGuess(initController);
    // nloc_17.solve();
    // solution = nloc_17.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_18(ocp, nloc_settings);
    // nloc_18.setInitialGuess(initController);
    // nloc_18.solve();
    // solution = nloc_6.getSolution();

    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_18.setInitialGuess(initController);
    // nloc_18.solve();
    // solution = nloc_18.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_19(ocp, nloc_settings);
    // nloc_19.setInitialGuess(initController);
    // nloc_19.solve();
    // solution = nloc_19.getSolution();
    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_19.setInitialGuess(initController);
    // nloc_19.solve();
    // solution = nloc_19.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_20(ocp, nloc_settings);
    // nloc_20.setInitialGuess(initController);
    // nloc_20.solve();
    // solution = nloc_20.getSolution();
    // *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_20.setInitialGuess(initController);
    // nloc_20.solve();
    // solution = nloc_20.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_21(ocp, nloc_settings);
    // nloc_21.setInitialGuess(initController);
    // nloc_21.solve();
    // solution = nloc_21.getSolution();
    //     *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_21.setInitialGuess(initController);
    // nloc_21.solve();
    // solution = nloc_21.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_23(ocp, nloc_settings);
    // nloc_23.setInitialGuess(initController);
    // nloc_23.solve();
    // solution = nloc_23.getSolution();
    //     *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_23.setInitialGuess(initController);
    // nloc_23.solve();
    // solution = nloc_23.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_24(ocp, nloc_settings);
    // nloc_24.setInitialGuess(initController);
    // nloc_24.solve();
    // solution = nloc_24.getSolution();
    //     *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_24.setInitialGuess(initController);
    // nloc_24.solve();
    // solution = nloc_24.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // //test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);

    // ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc_25(ocp, nloc_settings);
    // nloc_25.setInitialGuess(initController);
    // nloc_25.solve();
    // solution = nloc_25.getSolution();
    //     *previous_optimal_states = solution.x_ref();
    // initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
    // nloc_25.setInitialGuess(initController);
    // nloc_25.solve();
    // solution = nloc_25.getSolution();
    // vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
    // test_track->plot_track(solution.x_ref());

    // *previous_optimal_states = solution.x_ref();
    // (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
    // (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[48]);
    // u0_ff = solution.uff();
    // u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
    // u0_ff.toImplementation().push_back((u0_ff.toImplementation())[47]);
    // initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
    // ocp.setInitialState(x0);
    int i_loop(0);
    while (true)
    {
        nloc.setInitialGuess(initController);
        nloc.prepareMPCIteration();
        nloc.changeInitialState(x0);
        nloc.finishMPCIteration();

        for (size_t k = 0; k < 1; k++)
        {
            solution = nloc.getSolution();
            initController.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
            nloc.setInitialGuess(initController);
            *previous_optimal_states = solution.x_ref();
            nloc.prepareMPCIteration();
            nloc.finishMPCIteration();
            solution = nloc.getSolution();
        }
        vehicle_dynamic->propagateControlledDynamics(x0, 0, (solution.uff())[0], x0);
        x0(4) = 0;
        x0(5) = 0;

        *previous_optimal_states = solution.x_ref();
        (previous_optimal_states->toImplementation()).erase(previous_optimal_states->toImplementation().begin());
        (previous_optimal_states->toImplementation()).push_back((previous_optimal_states->toImplementation())[horizon - 1]);

        u0_ff = solution.uff();
        u0_ff[1] += u0_ff[0];
        u0_ff.toImplementation().erase(u0_ff.toImplementation().begin());
        u0_ff.toImplementation().push_back((u0_ff.toImplementation())[horizon - 2]);
        for (auto u0_ff_ : u0_ff)
            u0_ff_[2] = 0;
        initController.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());


        // for(auto x_opt : *previous_optimal_states)
        //     std::cout << x_opt.transpose() << std::endl;
        if (i_loop % 10 == 0)
        {
            test_track->plot_track(solution.x_ref());

        }
        i_loop++;
    }


	return 0;
}

