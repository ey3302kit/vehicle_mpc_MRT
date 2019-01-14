/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


#include <Eigen/Core>
#include <map>


#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/spline/SplinerBase.h>
#include <ct/optcon/dms/dms_core/spline/ZeroOrderHold/ZeroOrderHoldSpliner.h>
#include <ct/optcon/dms/dms_core/spline/Linear/LinearSpliner.h>
#include <ct/optcon/nlp/OptVector.h>
#include <ct/optcon/dms/dms_core/DmsSettings.h>


namespace ct {
namespace optcon {


/**
 * @ingroup    DMS
 *
 * @brief      This class is a wrapper around the NLP Optvector. It wraps the
 *             Vectors from the NLP solvers into state, control and time
 *             trajectories
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class OptVectorDmsVehicle : public tpl::OptVector<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;
    typedef tpl::OptVector<SCALAR> Base;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

    /**
     * @brief      Custom constructor
     *
     * @param[in]  n         The number of optimization variables
     * @param[in]  settings  The dms settings
     */
    OptVectorDmsVehicle(const size_t & n, const size_t & N_)
        : tpl::OptVector<SCALAR>(n), numPairs_(N_ + 1)
    {
        size_t currIndex = 0;

        for (size_t i = 0; i < numPairs_; i++)
        {
            pairNumToStateIdx_.insert(std::make_pair(i, currIndex));
            currIndex += STATE_DIM;

            pairNumToControlIdx_.insert(std::make_pair(i, currIndex));
            currIndex += CONTROL_DIM;
        }

        stateSolution_.resize(numPairs_);
        inputSolution_.resize(numPairs_);
    }

    OptVectorDmsVehicle(const OptVectorDmsVehicle& arg) = delete;

    /**
     * @brief      Destructor
     */
    ~OptVectorDmsVehicle() override = default;

    /**
     * @brief      Returns the optimized state for a specific shot
     *
     * @param[in]  pairNum  The shot number
     *
     * @return     The optimized state
    */
    state_vector_t getOptimizedState(const size_t pairNum) const
    {
        size_t index = getStateIndex(pairNum);
        return (this->x_.segment(index, STATE_DIM));
    }

    /**
     * @brief      Returns the optimized control input for a specific shot
     *
     * @param[in]  pairNum  The shot number
     *
     * @return     The optimized control input
    */
    control_vector_t getOptimizedControl(const size_t pairNum) const
    {
        size_t index = getControlIndex(pairNum);
        return (this->x_.segment(index, CONTROL_DIM));
    }

    SCALAR getXi()
    {
        return this->x_(numPairs_ * (STATE_DIM + CONTROL_DIM));
    }

    /**
     * @brief      Returns the optimized state for all shots
     *
     * @return     The optimized states.
    */
    const state_vector_array_t& getOptimizedStates()
    {
        for (size_t i = 0; i < numPairs_; i++)
        stateSolution_[i] = getOptimizedState(i);

        return stateSolution_;        
    }

    /**
     * @brief      Returns the optimized control inputs for all shots
     *
     * @return     The optimized control inputs.
    */
    const control_vector_array_t& getOptimizedInputs()
    {
        for (size_t i = 0; i < numPairs_; i++)
        inputSolution_[i] = getOptimizedControl(i);

        return inputSolution_;
    }

    /**
     * @brief      Returns the starting index for the state at shot pairNum
     *             inside the optimization vector
     *
     * @param[in]  pairNum  The shot number
     *
     * @return     The state index.
     */
    size_t getStateIndex(const size_t pairNum) const
    {
        return pairNumToStateIdx_.find(pairNum)->second;
    }

    /**
     * @brief      Returns the starting index for the control input at shot pairNum
     *             inside the optimization vector
     *
     * @param[in]  pairNum  The shot number
     *
     * @return     The state index.
     */
    size_t getControlIndex(const size_t pairNum) const
    {
        return pairNumToControlIdx_.find(pairNum)->second;
    }

    /**
     * @brief      Sets an initial guess for the optimal solution. The optimal
     *             solution is set as a linear interpolation between inital
     *             state x0 and final state xf. The initial guess for the
     *             control input is assumed to be constant and equal to u0
     *
     * @param[in]  x0    The initial state
     * @param[in]  x_f   The final state
     * @param[in]  u0    The control input
     */
    // void setInitGuess(const state_vector_t& x0, const state_vector_t& x_f, const control_vector_t& u0);

    /**
     * @brief      Sets an initial guess for the optimal solution.
     *
     * @param[in]  x_init  Initial guess for the state trajectory
     * @param[in]  u_init  Initial guess for the control trajectory
     */
    void setInitGuess(const state_vector_array_t& x_init, const control_vector_array_t& u_init, const SCALAR & xi_init)
    {
        if (x_init.size() != numPairs_)
            throw std::runtime_error("initial guess state trajectory not matching number of shots");
        if (u_init.size() != numPairs_)
            throw std::runtime_error("initial guess input trajectory not matching number of shots");

        for (size_t i = 0; i < numPairs_; i++)
        {
            size_t s_index = getStateIndex(i);
            size_t q_index = getControlIndex(i);

            this->xInit_.segment(s_index, STATE_DIM) = x_init[i];
            this->xInit_.segment(q_index, CONTROL_DIM) = u_init[i];
        }

        this->xInit_(numPairs_ * (STATE_DIM + CONTROL_DIM)) = xi_init;
    }

    /**
     * @brief      Updates the initial state
     *
     * @param[in]  x0    The new initial state
     */
    //void changeInitialState(const state_vector_t& x0);

    /**
     * @brief      Updates the final state
     *
     * @param[in]  xF    The new final state
     */
    //void changeDesiredState(const state_vector_t& xF);

    /**
     * @brief      Returns the number of pairs 
     *
     * @return     Number of pairs
     */
    size_t numPairs() { return numPairs_; }
    /**
     * @brief      Prints out the solution trajectories
     */
    void printoutSolution()
    {
            std::cout << "... printing solutions: " << std::endl;
        std::cout << "x_solution" << std::endl;
        state_vector_array_t x_sol = getOptimizedStates();
        for (size_t i = 0; i < x_sol.size(); ++i)
        {
            std::cout << x_sol[i].transpose() << std::endl;
        }

        std::cout << "u_solution" << std::endl;
        control_vector_array_t u_sol = getOptimizedInputs();
        for (size_t i = 0; i < u_sol.size(); ++i)
        {
            std::cout << u_sol[i].transpose() << std::endl;
        }

        std::cout << "xi_solution" << std::endl;

        std::cout << this->getXi() << std::endl;

        std::cout << std::endl;
        std::cout << " ... done." << std::endl;
    }

    typename Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> getX()
    {
        return this->x_;
    }

private:
    DmsSettings settings_;

    const size_t numPairs_;
    /* maps the number of a "pair" to the index in w where ... */
    std::map<size_t, size_t> pairNumToStateIdx_;   /* ... its state starts */
    std::map<size_t, size_t> pairNumToControlIdx_; /* ... its control starts */

    state_vector_array_t stateSolution_;
    control_vector_array_t inputSolution_;
};

}  // namespace optcon
}  // namespace ct
