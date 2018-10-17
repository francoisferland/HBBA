#ifndef SOLVER_IMPL_HPP
#define SOLVER_IMPL_HPP
 
// OR-Tools generates tons of warnings from its include files, disable them
// temporarily.
// Another one (no-deprecated) has been forced in the build config.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <ortools/constraint_solver/constraint_solver.h>
#pragma GCC diagnostic pop

#include <boost/scoped_ptr.hpp>

namespace iw_translator
{
    /// \brief Implementation class - used to isolate or_tools dependencies (and
    /// multiple compilation warnings).
    struct SolverImpl
    {
        typedef std::vector<int64>                             IntVector;
        typedef operations_research::IntVar                    IntVar;
        typedef std::vector<IntVar*>                           IntVarVector;
        typedef std::vector<operations_research::Constraint*>  ConstraintsVector;
        typedef boost::scoped_ptr<operations_research::Solver> ORSolverPtr;

        IntVarVector      a;
        ORSolverPtr       or_solver;

    };
}

#endif

