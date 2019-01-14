#include <ct/core/core.h>
#include <Eigen/Dense>
#include <ctime>
#include <iostream>

#include "Vehicle.h"
#include "IntermediateGeneralAccelerationConstraints_Vehicle.h"
#include "TestTrack.h"
#include "get_AccelerationConstraints.h"
#include "IntermediateGeneralPositionConstraints_Vehicle.h"
#include "TerminalLinearTerm_Vehicle.h"
#include "TerminalConstraintVehicle.h"

static const size_t state_dim = 6;
static const size_t control_dim = 3;
static const std::shared_ptr<TestTrack<state_dim, control_dim>> test_track(new TestTrack<state_dim, control_dim>());
static const std::shared_ptr<AccelerationLimits<state_dim, control_dim>> acceleration_limits(new AccelerationLimits<state_dim, control_dim>());
static const size_t horizon = 50;
static const bool verbose = false;
static const size_t internal_loop = 2;

int main(int argc, char const *argv[])
{
    /*
        initial state
    */
    ct::core::StateVector<state_dim, double> x0;
    x0 << 0, 0, 0, 0, 0, 0;

    /*
        previous optimal states
    */
	std::shared_ptr<ct::core::StateVectorArray<state_dim, double>> previous_optimal_states(
		new ct::core::StateVectorArray<state_dim, double>(horizon + 1));
    previous_optimal_states->setConstant(x0);


	std::shared_ptr<ct::core::DiscreteControlledSystem<state_dim, control_dim, double>> vehicle_dynamic(
		new ct::core::Vehicle<double>(0.1));

	std::shared_ptr<ct::core::DiscreteSystemLinearizer<state_dim, control_dim>> vehicle_linearizer(
		new ct::core::DiscreteSystemLinearizer<state_dim, control_dim>(vehicle_dynamic));

	/*
        cost function
    */
    // the term du^T * R * du works after the 1st stage
    std::shared_ptr<ct::core::tpl::SingleActivation<double>> activation(
        new ct::core::tpl::SingleActivation<double>(0.05, 1000));

    Eigen::Matrix<double, state_dim, state_dim> Q(Eigen::Matrix<double, state_dim, state_dim>::Zero());
    Eigen::Matrix<double, control_dim, control_dim> R(Eigen::Matrix<double, control_dim, control_dim>::Zero());
    R(0, 0) = 0.25;
    R(1, 1) = 0.25;
    R(2, 2) = 1e8;

    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediate_cost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>(Q, R));
    intermediate_cost->setTimeActivation(activation, verbose);

    // the term xi^T * R * xi begins working at 0st stage
    std::shared_ptr<ct::core::tpl::SingleActivation<double>> activation_slack_variable(
        new ct::core::tpl::SingleActivation<double>(-1, 0.05));

    Eigen::Matrix<double, control_dim, control_dim> R_slack_variable(Eigen::Matrix<double, control_dim, control_dim>::Zero());
    R_slack_variable(2, 2) = 1e8;

    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediate_cost_slack_variable(
        new ct::optcon::TermQuadratic<state_dim, control_dim>(Q, R_slack_variable));
    intermediate_cost_slack_variable->setTimeActivation(activation_slack_variable, verbose);

    // final term in cost function
    std::shared_ptr<ct::optcon::TerminalLinearTerm_Vehicle<state_dim, control_dim>> final_cost(
        new ct::optcon::TerminalLinearTerm_Vehicle<state_dim, control_dim>(test_track, previous_optimal_states));


    std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>> cost_function(
        new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());
    cost_function->addIntermediateTerm(intermediate_cost);
    cost_function->addIntermediateTerm(intermediate_cost_slack_variable);
    cost_function->addFinalTerm(final_cost);

    
    // constraints
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> general_constraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> box_constraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>> intermediate_general_acceleration_constraint(
		new ct::optcon::IntermediateGeneralAccelerationConstraints_Vehicle<state_dim, control_dim, double>(previous_optimal_states, acceleration_limits, 0.1));

    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>> intermediate_general_position_constraint(
        new ct::optcon::IntermediateGeneralPositionConstraints_Vehicle<state_dim, control_dim, double>(previous_optimal_states, test_track, 0.1));
    
    // state box constraints
    Eigen::Matrix<double, 2, 1> x_u_b;
    Eigen::Matrix<double, 2, 1> x_l_b;
    x_u_b << 40, 40;
    x_l_b << -40, -40;
    Eigen::Matrix<int, state_dim, 1> sparse;
    sparse << 0, 0, 0, 0, 1, 1;
    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>> intermediate_box_state_constraint(
        new ct::optcon::StateConstraint<state_dim, control_dim, double>(x_l_b, x_u_b, sparse));

    // input box constraints
    Eigen::Matrix<double, 3, 1> u_u_b;
    Eigen::Matrix<double, 3, 1> u_l_b;
    u_u_b << 40, 40, 1e5;
    u_l_b << -40, -40, 0;
    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>> intermediate_box_control_constraint(
        new ct::optcon::ControlInputConstraint<state_dim, control_dim, double>(u_l_b, u_u_b));
    
    // terminal box constraint
    Eigen::Matrix<double, 2, 1> x_b_terminal(Eigen::Matrix<double, 2, 1>::Zero());
    Eigen::Matrix<int, state_dim, 1> sparse_terminal;
    sparse_terminal << 0, 0, 1, 1, 0, 0; 
    std::shared_ptr<ct::optcon::ConstraintBase<state_dim, control_dim, double>> terminal_box_state_constraint(
        new ct::optcon::StateConstraint<state_dim, control_dim, double>(x_b_terminal, x_b_terminal, sparse_terminal));


    general_constraints->addIntermediateConstraint(intermediate_general_acceleration_constraint, verbose);
    general_constraints->addIntermediateConstraint(intermediate_general_position_constraint, verbose);
    general_constraints->addTerminalConstraint(intermediate_general_acceleration_constraint, verbose);
    general_constraints->addTerminalConstraint(intermediate_general_position_constraint, verbose);

    box_constraints->addIntermediateConstraint(intermediate_box_control_constraint, verbose);
    box_constraints->addIntermediateConstraint(intermediate_box_state_constraint, verbose);
    box_constraints->addTerminalConstraint(terminal_box_state_constraint, verbose);
    //boxConstraints->addTerminalConstraint(intermediate_box_control_constraint, true);
    box_constraints->addTerminalConstraint(intermediate_box_state_constraint, verbose);

    general_constraints->initialize();
    box_constraints->initialize();

    /*
        ocp
    */
    ct::optcon::DiscreteOptConProblem<state_dim, control_dim> ocp(
    	horizon, x0, vehicle_dynamic, cost_function, vehicle_linearizer);

    ocp.setGeneralConstraints(general_constraints);
    ocp.setBoxConstraints(box_constraints);

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
    nloc_settings.lqoc_solver_settings.lqoc_debug_print = false;
    nloc_settings.lineSearchSettings.active = true;
    nloc_settings.lineSearchSettings.debugPrint = false;
    nloc_settings.printSummary = false;

    /*
        mpc
    */
    ct::core::FeedbackArray<state_dim, control_dim> u0_fb(horizon , ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());
    ct::core::ControlVectorArray<control_dim> u0_ff(horizon, ct::core::ControlVector<control_dim>::Zero());
    ct::core::StateVectorArray<state_dim> x_ref_init(horizon + 1, x0);

    ct::optcon::NLOptConSolver<state_dim, control_dim>::Policy_t init_guess(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);
    ct::optcon::NLOptConSolver<state_dim, control_dim, 3, 3, double, false> nloc(ocp, nloc_settings);
    ct::core::StateFeedbackController<state_dim, control_dim> solution;

    
    int i_loop(0);
    clock_t start, finish;
    while (true)
    {
        start = clock();

        for (size_t k = 0; k < internal_loop; k++)
        {
            //solution = nloc.getSolution();
            nloc.setInitialGuess(init_guess);
            nloc.prepareMPCIteration();
            nloc.changeInitialState(x0);
            nloc.finishMPCIteration();

            solution = nloc.getSolution();

            init_guess.update(solution.x_ref(), solution.uff(), solution.K(), solution.time());
            *previous_optimal_states = solution.x_ref();
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
        init_guess.update(*previous_optimal_states, u0_ff, solution.K(), solution.time());
        
        finish = clock();
        std::cout << "The run time of iteration " << i_loop << " is: " << double(finish - start) / CLOCKS_PER_SEC << "s." << std::endl;

        if (i_loop % 10 == 0)
        {
            test_track->plot_Track(solution.x_ref());

        }
        i_loop++;
    }


	return 0;
}

