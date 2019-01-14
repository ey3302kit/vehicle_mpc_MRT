#include <ct/optcon/optcon.h>
#include <Eigen/Dense>
#include <iostream>

#include "DmsProblemVehicle.h"
#include "testTrack1.h"

void plotTrack(ct::core::StateVectorArray<4, double> xSolution_)
{
	CheckPoints cp;
	cp = testTrack1();

	using namespace ct::core;

	plot::ion();
	plot::figure();


	plot::plot(cp.left.row(0).transpose(), cp.left.row(1).transpose());
	plot::plot(cp.right.row(0).transpose(), cp.right.row(1).transpose());

	std::vector<double> x;
	std::vector<double> y;
	for (auto state : xSolution_)
	{
		x.push_back(state(0));
		y.push_back(state(1));
	}

	plot::plot(x, y);

	plot::show();
}

int main(int argc, char const *argv[])
{	// define the system
	const size_t state_dim = 4;
	const size_t control_dim = 2;
	Eigen::Matrix<double, state_dim, state_dim> A;
	Eigen::Matrix<double, state_dim, control_dim> B;
	A << 1, 0, 0.1, 0,
			0, 1, 0, 0.1,
			0, 0, 1, 0,
			0, 0, 0, 1;

	B << 0.005, 0,
			0, 0.005,
			0.1, 0,
			0, 0.1;


	// shoting intervals 
	const size_t N = 49;

	// weighting parameters
	Eigen::Matrix<double, 2, 2> R;
	R << 0.05, 0, 0.05, 0;

	double q(1e5);

	// parameters for the updating of the problem
	ct::core::StateVector<state_dim, double> x_0;
	x_0.setConstant(0.0);

	ct::core::StateVectorArray<state_dim> PreviousOptimalStates;
	PreviousOptimalStates.resize(N + 1, ct::core::StateVector<state_dim>::Zero());

	ct::core::ControlVectorArray<control_dim> PreviousOptimalInputs;
	PreviousOptimalInputs.resize(N + 1, ct::core::ControlVector<control_dim>::Zero());


	// define the initial guesses
	ct::core::StateVectorArray<state_dim> x_init_guess;
	ct::core::ControlVectorArray<control_dim> u_init_guess;
	double xi_init_guess(0);
	x_init_guess.resize(N + 1, ct::core::StateVector<state_dim>::Zero());
	u_init_guess.resize(N + 1, ct::core::ControlVector<control_dim>::Zero());

	// define the ocp
	auto problem = std::shared_ptr<ct::optcon::DmsProblemVehicle<state_dim, control_dim, double>>(
		new ct::optcon::DmsProblemVehicle<state_dim, control_dim, double>(N, R, q));

	// solver settings
	ct::optcon::NlpSolverSettings settings;
	settings.ipoptSettings_.max_iter_ = 200;
	settings.ipoptSettings_.tol_ = 1e-3;
    settings.ipoptSettings_.acceptableTol_=1e-3;
	// define the IpoptSolver
	ct::optcon::tpl::IpoptSolver<double> solver_vehicle(problem, settings);

	// MPC LOOP
	size_t it;
	std::cin >> it;

	for (size_t i = 0; i < it; i++)
	{
		problem->updateProblem(PreviousOptimalStates, x_0);

		problem->setInitialGuess(x_init_guess, u_init_guess, xi_init_guess);

		solver_vehicle.solve();

		PreviousOptimalStates = problem->getStateSolution();
		PreviousOptimalInputs = problem->getInputSolution();
		
		x_0 = A * x_0 + B * PreviousOptimalInputs[0];

		for (size_t i; i < N; ++i)
		{
			x_init_guess[i] = PreviousOptimalStates[i + 1];
			u_init_guess[i] = PreviousOptimalInputs[i + 1];
		}
		xi_init_guess = problem->getXi();

		if (i % 5 == 0)
			plotTrack(PreviousOptimalStates);
	}



	return 0;
}