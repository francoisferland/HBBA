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

    const Matrix&               u   = solver_model.u();
    const Matrix&               c   = solver_model.c();
    const Vector&               m   = solver_model.m();
    const Matrix&               ur  = solver_model.ur();
    SolverImpl::IntVarVector&   a   = impl_->a;

    // Total utility produced per class (a_ik * (u_ik - r_ik)):
    SolverImpl::IntVarVector aur;

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
        const MatrixColView   col = ur[boost::indices[MatrixRange()][k]];
        SolverImpl::IntVector ur_k(nb_strats);

        std::copy(col.begin(), col.end(), ur_k.begin());

        // Keep a copy of the scalar product for optimization.
        aur.push_back(solver.MakeScalProd(a, ur_k)->Var());

        // Add ">=" constraints for positive goals, as we don't mind if we
        // get higher utility than required.
        // However, add "<=" constraints for null (or negative) goals, as we
        // don't want to produce utility for unwanted desires.
        if (g[k] > 0.0) {
            solver.AddConstraint(
                solver.MakeScalProdGreaterOrEqual(
                    a, 
                    ur_k, 
                    g[k]));
        } else {
            // TODO: CURRENTLY DISABLED, BREAKS DEPENDENCIES GATHERING:
            // solver.AddConstraint(
            //    solver.MakeScalProdLessOrEqual(
            //        a,
            //        ur_k,
            //        g[k]));
        }
    }

    // Search monitors used for optimization (see Solve(...) call).
    std::vector<or_tools::SearchMonitor*> monitors;

    // Optimize for maximum utility production (non-optional).
    or_tools::IntVar* sum_aur = solver.MakeSum(aur)->Var();
    monitors.push_back(solver.MakeMaximize(sum_aur, 1));

    // Finally, minimize total count of activated strategies.
    // TODO: Remove this, probably no longer necessary with the '<='
    // constraints, and doesn't really mean anything.
    or_tools::IntVar* sum_a = solver.MakeSum(a)->Var();
    monitors.push_back(solver.MakeMinimize(sum_a, 1));

    // Choose a random unbound variable to start, start with minimum values
    // (deactivated strategies):
    or_tools::DecisionBuilder* const db = solver.MakePhase(
       a, 
       or_tools::Solver::CHOOSE_RANDOM,
       or_tools::Solver::ASSIGN_MIN_VALUE);

    or_tools::SolutionCollector* coll = 
        solver.MakeBestValueSolutionCollector(false);
    coll->Add(a);
    coll->AddObjective(sum_a);
    coll->AddObjective(sum_aur);
    monitors.push_back(coll);

    solver.NewSearch(db, coll);
    solver.Solve(db, monitors);

    int nb_sols = coll->solution_count();
    ROS_DEBUG("Solutions count: %i", nb_sols);

    if (nb_sols == 0) {
        ROS_ERROR("IW Solver could not find a solution.");
        a_res_.clear();
    } else {
        a_res_.resize(a.size());
        for (size_t i = 0; i < a_res_.size(); ++i) {
            int64 v = coll->Value(0, a[i]);
            a_res_[i] = (v > 0);
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

