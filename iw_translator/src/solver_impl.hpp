#ifndef SOLVER_IMPL_HPP
#define SOLVER_IMPL_HPP
 
// OR-Tools generates tons of warnings from its include files, disable them
// temporarily.
// Another one (no-deprecated) has been forced in the build config.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <constraint_solver/constraint_solver.h>
#pragma GCC diagnostic pop

#include <boost/scoped_ptr.hpp>

namespace iw_translator
{
    /// \brief Implementation class - used to isolate or_tools dependencies (and
    /// multiple compilation warnings).
    struct SolverImpl
    {
        boost::scoped_ptr<operations_research::Solver> or_solver;
        
        typedef operations_research::IntVar          IntVar;
        typedef operations_research::vector<IntVar*> IntVarVector;

        IntVarVector a;
    };
}

#endif

