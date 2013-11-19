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

    /// \brief Solver configuration structure.
    struct SolverParams
    {
        /// \brief If total utility production for defined, positive, and 
        /// activated should be done.
        /// Default: false.
        bool max_p;

        /// \brief Search time limit, in ms.
        /// Default: 0 (no limit)
        int time_limit;

        /// \brief Perform simulated annealing in solution search.
        /// Default: false.
        bool sa;

        /// \brief If search should be logged to the standard output.
        /// Default: false.
        bool log;

        SolverParams():
            max_p(false),
            time_limit(0),
            sa(false),
            log(false)
        {
        }
    };

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
        /// \param params       A SolverParams instance that sets different
        ///                     solving options.
        Solver(const SolverModel&  solver_model, 
               const Vector&       g,
               const Vector&       s,
               const SolverParams& params = SolverParams());

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

