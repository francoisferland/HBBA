#include "solver_impl.hpp"

#include <iw_translator/solver.hpp>

using namespace iw_translator;

Solver::Solver(const SolverModel& solver_model): 
    impl_(new SolverImpl())
{
    namespace or_tools = operations_research;

    typedef Matrix::const_array_view<1>::type     ColView;
    typedef boost::multi_array_types::index_range Range;
    typedef std::vector<int64>                    IntVector;

    impl_->or_solver.reset(new or_tools::Solver("iw_solver_impl"));

    or_tools::Solver& solver = *(impl_->or_solver);

    const Matrix&               u = solver_model.u();
    const Matrix&               c = solver_model.c();
    const Matrix&               r = solver_model.r();
    const Vector&               m = solver_model.m();
    SolverImpl::IntVarVector&   a = impl_->a;

    size_t nb_strats = u.size();
    size_t nb_res    = m.size();
    size_t nb_cls    = u.shape()[1];

    // Result vector (strategy activation vector A):
    solver.MakeIntVarArray(nb_strats, 0, 1, &a);

    // Convert M vector (maximum cost allowed) to constraints.
    for (size_t j = 0; j < nb_res; ++j) {
        const ColView col = c[boost::indices[Range()][j]];
        IntVector c_j(nb_strats); 
        std::copy(col.begin(), col.end(), c_j.begin());

        solver.AddConstraint(solver.MakeScalProdLessOrEqual(a, c_j, m[j]));
    }


    
}

Solver::~Solver()
{
    delete impl_;
}

