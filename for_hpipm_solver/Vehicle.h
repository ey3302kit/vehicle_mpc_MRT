#pragma once
#include <ct/core/core.h>
#include <Eigen/Dense>

/*
	The original dynamic of the vehicle is x(k+1) = A*x(k) + B*u(k).
	In this project the dynamic is modified to:
	x(k+1) = A B * x(k) + B 0 * du(k)
	u(k+1)   0 I   u(k)	  I 0	xi(k)
	xi is slack variable. It doesn't affect the dynamic of vehicle.It is introduced because CT HPIPMInterface doesn't support slack variables.
	Then the total state number is 6 and input number is 3.
*/

namespace ct{
namespace core{

template <typename SCALAR>
class Vehicle : public DiscreteControlledSystem<6, 3, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DiscreteControlledSystem<6, 3, SCALAR> Base;
	typedef typename Base::state_vector_t state_vector_t;
	typedef typename Base::control_vector_t control_vector_t;
	typedef typename Base::time_t time_t;

	/*
		param: sampling time (0.1s in the project)
	*/
	Vehicle(const double& delta_t) : delta_t_(delta_t)
	{
		const size_t real_state_dim(4);	
		const size_t real_control_dim(2);
		const size_t state_dim_(6);
		const size_t control_dim_(3);

		A_.resize(state_dim_, state_dim_);
		B_.resize(state_dim_, control_dim_);

		A_.setConstant(SCALAR(0.));
		B_.setConstant(SCALAR(0.));


		Eigen::Matrix<double, real_state_dim, real_state_dim> A;
		Eigen::Matrix<double, real_state_dim, real_control_dim> B;
		Eigen::Matrix<double, real_control_dim, real_control_dim> E( Eigen::Matrix<double, real_control_dim, real_control_dim>::Identity() );
		Eigen::Matrix<double, 1, 1> E_slack(Eigen::Matrix<double, 1, 1>::Identity());

		A << 1, 0, delta_t_, 0,
			 0, 1, 0, delta_t_,
			 0, 0, 1, 0,
			 0, 0, 0, 1;

		B << delta_t_ * delta_t_ / 2., 0,
			 0, delta_t_ * delta_t_ / 2.,
			 delta_t_, 0,
			 0, delta_t_;

		A_.block(0, 0, real_state_dim, real_state_dim) = A.template cast<SCALAR>();
		A_.block(0, real_state_dim, real_state_dim, real_control_dim) = B.template cast<SCALAR>();
		A_.block(real_state_dim, real_state_dim, real_control_dim, real_control_dim) = E.template cast<SCALAR>();

		B_.block(0, 0, real_state_dim, real_control_dim) = B.template cast<SCALAR>();
		B_.block(real_state_dim, 0, real_control_dim, real_control_dim) = E.template cast<SCALAR>();
	}

	Vehicle(const Vehicle<SCALAR>* arg) : delta_t_(arg.delta_t_),
		A_(arg.A_),
		B_(arg.B_),
		DiscreteControlledSystem<6, 2, SCALAR>(arg) 
		{}

	virtual Vehicle<SCALAR>* clone () const override { return new Vehicle<SCALAR>(*this); }

	virtual ~Vehicle() = default;

	virtual void propagateControlledDynamics(const state_vector_t& state, 
		const time_t n,
		const control_vector_t& u,
		state_vector_t& stateNext)
	{
			stateNext = A_ * state + B_ * u;
	}
private:
	double delta_t_;

	Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> A_;
	Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> B_;
};

}
}