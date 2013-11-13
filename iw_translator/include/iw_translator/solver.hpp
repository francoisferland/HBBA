#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <iw_translator/solver_model.hpp>
#include <boost/scoped_ptr.hpp>

namespace iw_translator
{

    // Forward declartion of isolated implementation class:
    class SolverImpl;

    /// \brief A vector for strategy activation results (bool)
    typedef std::vector<bool> ActivationVector;

    /// \brief A Google OR Tools-based solver for the Intention Workspace.
    ///
    /// Work based on the previous version of the solver (iw_solver_ortools).
    /// Meant to work with the new SolverModel class.
    ///
    ///
    class Solver
    {
    private:
        SolverImpl*      impl_;
        ActivationVector a_res_;


    public:
        /// \brief Constructor.
        ///
        /// Search a solution according to both the static model and the goal 
        /// parameters.
        /// The result, if available, can be obtained with result().
        ///
        /// \param solver_model The static solver model that serves as a basis
        ///                     for the solver.
        /// \param g            The required utility vector, or goal (G)
        /// \param s            The intensity vector (S)
        /// \param max_p        If it should maximise utility production.
        ///                     False by default.
        Solver(const SolverModel& solver_model, 
               const Vector&      g,
               const Vector&      s,
               const bool         max_p = false);

        /// \brief Destructor.
        ~Solver();

        /// \brief Copy the strategy activation vector (a), if available.
        ///
        /// Will return the first (or best, depending on the optimization
        /// options) solution available.
        ///
        /// \param  a Output vector, will be resized, cleared and filled only if
        ///         a solution could be found.
        /// \return False If an error occured while solving or a solution
        ///         could not be found.
        bool result(ActivationVector& a);

    };
}

#endif

