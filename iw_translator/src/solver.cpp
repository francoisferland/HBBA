#include "solver_impl.hpp"

#include <iw_translator/solver.hpp>

using namespace iw_translator;

Solver::Solver(const SolverModel& solver_model, const Vector& g): 
    impl_(new SolverImpl())
{
    namespace or_tools = operations_research;

    std::stringstream gstr;
    gstr << "G: [ ";
    for (size_t k = 0; k < g.size(); ++k) {
        gstr << g[k] << " ";
    }
    gstr << "]";
    ROS_DEBUG("%s", gstr.str().c_str());

    impl_->or_solver.reset(new or_tools::Solver("iw_solver_impl"));

    or_tools::Solver& solver = *(impl_->or_solver);

    const Matrix&               u  = solver_model.u();
    const Matrix&               c  = solver_model.c();
    const Vector&               m  = solver_model.m();
    const Matrix&               ur = solver_model.ur();
    SolverImpl::IntVarVector&   a  = impl_->a;

    size_t nb_strats = ur.shape()[0];
    size_t nb_res    = m.size();
    size_t nb_cls    = ur.shape()[1];

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
        // Do not add constraints for zero goals, avoid activating strategies
        // for undesired classes.
        // TODO: Check first if it's really needed.
        // if (!(g[k] > 0.0)) {
        //     continue;
        // }

        const MatrixColView   col = ur[boost::indices[MatrixRange()][k]];
        SolverImpl::IntVector ur_k(nb_strats);

        std::copy(col.begin(), col.end(), ur_k.begin());

        solver.AddConstraint(
            solver.MakeScalProdGreaterOrEqual(
                a, 
                ur_k, 
                g[k]));
    }

    or_tools::DecisionBuilder* const db = solver.MakePhase(
       a, 
       or_tools::Solver::CHOOSE_RANDOM,
       or_tools::Solver::ASSIGN_MAX_VALUE);

    // TODO: Add monitors for optimizations ?
    solver.NewSearch(db);

    // Only keep the first solution if available.
    if (!solver.NextSolution()) {
        ROS_ERROR("IW Solver could not find solution.");
        a_res_.clear();
        //return false;
    } else {
        a_res_.resize(a.size());
        for (size_t i = 0; i < a_res_.size(); ++i) {
            a_res_[i] = (a[i]->Value() > 0);
        }
    }

    solver.EndSearch();

}

Solver::~Solver()
{
    delete impl_;
}

bool Solver::result(ActivationVector& a_res)
{
    if (a_res_.empty()) {
        return false;
    } else {
        a_res = a_res_;
        return true;
    }
}

