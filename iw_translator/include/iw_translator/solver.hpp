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
        SolverImpl* impl_;

    public:
        /// \brief Constructor.
        ///
        /// Initialize the solver according to both the static model and the
        /// goal parameters.
        /// The solver is then ready to run with solve().
        ///
        /// \param solver_model The static solver model that serves as a basis
        ///                     for the solver.
        /// \param g            The required utility vector, or goal (G)
        Solver(const SolverModel& solver_model, const Vector& g);

        /// \brief Destructor.
        ~Solver();

        /// \brief Produce the strategy activation vector (a).
        ///
        /// Will return the first (or best, depending on the optimization
        /// options) solution available.
        ///
        /// \param  a Output vector, will be resized, cleared and filled only if
        ///         a solution could be found.
        /// \return False If an error occured while solving or a solution
        ///         could not be found.
        bool solve(ActivationVector& a);

    };
}

#endif

