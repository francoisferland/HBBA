#include "solver_impl.hpp"

#include <iw_translator/solver.hpp>

using namespace iw_translator;

Solver::Solver(const SolverModel& solver_model, const Vector& g): 
    impl_(new SolverImpl())
{
    namespace or_tools = operations_research;

    impl_->or_solver.reset(new or_tools::Solver("iw_solver_impl"));

    or_tools::Solver& solver = *(impl_->or_solver);

    const Matrix&               u  = solver_model.u();
    const Matrix&               c  = solver_model.c();
    const Vector&               m  = solver_model.m();
    const Matrix&               ur = solver_model.u();
    SolverImpl::IntVarVector&   a  = impl_->a;

    size_t nb_strats = ur.shape()[0];
    size_t nb_res    = m.size();
    size_t nb_cls    = ur.shape()[1];

    impl_->fixed_constraints.reserve(nb_strats * nb_res);

    // Result vector (strategy activation vector A):
    solver.MakeIntVarArray(nb_strats, 0, 1, &a);

    // Convert M vector (maximum cost allowed) to constraints.
    for (size_t j = 0; j < nb_res; ++j) {
        const MatrixColView   col = c[boost::indices[MatrixRange()][j]];
        SolverImpl::IntVector c_j(nb_strats); 

        std::copy(col.begin(), col.end(), c_j.begin());

        solver.AddConstraint(
            solver.MakeScalProdLessOrEqual(
                a, 
                c_j, 
                m[j]));
    }

    // Convert both G and UR (utility both required and provided by 
    // strategies) to constraints:
    for (size_t k = 0; k < nb_cls; ++k) {
        const MatrixColView   col = ur[boost::indices[MatrixRange()][k]];
        SolverImpl::IntVector ur_k(nb_strats);

        std::copy(col.begin(), col.end(), ur_k.begin());

        solver.AddConstraint(
            solver.MakeScalProdGreaterOrEqual(
                a, 
                ur_k, 
                g[k]));
    }

}

Solver::~Solver()
{
    delete impl_;
}

bool Solver::solve(Vector& a_res)
{
    namespace or_tools = operations_research;

    or_tools::Solver&               solver = *(impl_->or_solver);
    const SolverImpl::IntVarVector& a      = impl_->a;

    or_tools::DecisionBuilder* const db = solver.MakePhase(
       a, 
       or_tools::Solver::CHOOSE_RANDOM,
       or_tools::Solver::ASSIGN_MAX_VALUE);

    // TODO: Add monitors for optimizations ?
    solver.NewSearch(db);

    // Only keep the first solution if available.
    if (!solver.NextSolution()) {
        ROS_ERROR("IW Solver could not find solution.");
        return false;
    }

    a_res.resize(a.size());
    for (size_t i = 0; i < a_res.size(); ++i) {
        a_res[i] = (a[i]->Value() != 0);
    }

    return true;
}

