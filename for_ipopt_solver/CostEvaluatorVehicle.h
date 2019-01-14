#pragma once
#include <ct/optcon/optcon.h>

#include "OptVectorDmsVehicle.h"
#include "testTrack1.h"
#include "get_CheckpointIndices.h"

namespace ct{
namespace optcon{

template <size_t STATE_DIM, size_t CONTROL_DIM,	typename SCALAR = double>
class CostEvaluatorVehicle : public tpl::DiscreteCostEvaluatorBase<SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

	CostEvaluatorVehicle(std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM, SCALAR>> w, 
		const size_t & N,
		const SCALAR & q,
		const Eigen::Matrix<double, 2, 2> & R)
	: w_(w),
	N_(N),
	R_(R),
	q_(q)
	{
	}

	~CostEvaluatorVehicle() override = default;

	SCALAR eval() override
	{	
		SCALAR cost = SCALAR(0.0);
		control_vector_t u_diff;

#pragma omp parallel for num_threads(settings_.nThreads_)

		// evaluate first term
		cost += -f_ * w_->getOptimizedState(N_).head(2);

		// evaluate second term
		cost += q_ * w_->getXi() * w_->getXi();

		for (size_t i = 1; i < N_ + 1; ++i)
		{
			u_diff = w_->getOptimizedControl(i) - w_->getOptimizedControl(i - 1);

			cost += u_diff.transpose() * R_ * u_diff;
		}


	}

	virtual void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override
	{	
		grad.setZero();


        for (size_t i = 0; i < N_ + 1; ++i)
        {
            if (i == 0)
                grad.segment(w_->getControlIndex(i), CONTROL_DIM) << -2 * R_(0, 0) * (w_->getOptimizedControl(i + 1)(0) - w_->getOptimizedControl(i)(0)),
                                                                    -2 * R_(1, 1) * (w_->getOptimizedControl(i + 1)(1) - w_->getOptimizedControl(i)(1));
            else if (i == N_)
            {
                grad.segment(w_->getStateIndex(i), 2) = -f_;
                grad.segment(w_->getControlIndex(i), CONTROL_DIM) << 2 * R_(0, 0) * (w_->getOptimizedControl(i)(0) - w_->getOptimizedControl(i - 1)(0)),
                                                                    2 * R_(1, 1) * (w_->getOptimizedControl(i)(1) - w_->getOptimizedControl(i - 1)(1));
            }

            else
                grad.segment(w_->getControlIndex(i), CONTROL_DIM) << R_(0, 0) * (-2 * (w_->getOptimizedControl(i + 1)(0) - w_->getOptimizedControl(i)(0)) + 2 * (w_->getOptimizedControl(i)(0) - w_->getOptimizedControl(i - 1)(0))),
                                                                    R_(1, 1) * (-2 * (w_->getOptimizedControl(i + 1)(1) - w_->getOptimizedControl(i)(1)) + 2 * (w_->getOptimizedControl(i)(1) - w_->getOptimizedControl(i - 1)(1)));
        }

        grad.tail(1) << 2 * q_ * w_->getXi();
	}

	void updateCost(const state_vector_array_t & PreviousOptimalStates)
	{
		CheckpointIndices_.resize(N_ + 1);
		CheckpointIndices_ = get_CheckpointIndices(PreviousOptimalStates, checkpoint);

		f_ = checkpoint.forward_vector.col(CheckpointIndices_(N_));
	}

private:
	std::shared_ptr<OptVectorDmsVehicle<STATE_DIM, CONTROL_DIM>> w_;
	Eigen::Matrix<size_t, Eigen::Dynamic, 1> CheckpointIndices_;
	size_t N_;

	Eigen::Matrix<double, 1, 2> f_;
	Eigen::Matrix<double, 2, 2> R_;
	SCALAR q_;

	const CheckPoints checkpoint = testTrack1();

};

}
}