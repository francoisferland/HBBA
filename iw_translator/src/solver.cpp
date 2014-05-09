#include "solver_impl.hpp"

#include <iw_translator/solver.hpp>
#include <ros/ros.h>

using namespace iw_translator;

Solver::Solver(
    const SolverModel& solver_model, 
    const Vector& g, 
    const Vector& s,
    const SolverParams& params): 
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
    const Matrix&               r   = solver_model.r();
    const Matrix&               c   = solver_model.c();
    const Vector&               m   = solver_model.m();
    const Matrix&               ur  = solver_model.ur();
    SolverImpl::IntVarVector&   a   = impl_->a;

    // Total (raw) utility production per class (a_ik * u_ik):
    SolverImpl::IntVarVector au;
    // Total utility produced per class (a_ik * (u_ik - r_ik)):
    SolverImpl::IntVarVector aur;

    size_t nb_strats = ur.shape()[0];
    size_t nb_res    = m.size();
    size_t nb_cls    = ur.shape()[1];

    au.reserve(nb_cls);
    aur.reserve(nb_cls);

    // Result vector (strategy activation vector A):
    solver.MakeIntVarArray(nb_strats, 0, 1, &a);

    // F vector for desire activation (in f_k * g_k)
    SolverImpl::IntVarVector f;
    solver.MakeIntVarArray(nb_cls, 0, 1, &f);
    
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
        const MatrixColView   u_col  = u[boost::indices[MatrixRange()][k]];
        const MatrixColView   r_col  = r[boost::indices[MatrixRange()][k]];
        const MatrixColView   ur_col = ur[boost::indices[MatrixRange()][k]];
        SolverImpl::IntVector u_k(nb_strats);
        SolverImpl::IntVector r_k(nb_strats);
        SolverImpl::IntVector ur_k(nb_strats);
        std::copy(u_col.begin(),  u_col.end(),  u_k.begin());
        std::copy(r_col.begin(),  r_col.end(),  r_k.begin());
        std::copy(ur_col.begin(), ur_col.end(), ur_k.begin());

        // Keep a copy of the scalar products for optimization.
        aur.push_back(solver.MakeScalProd(a, ur_k)->Var());
        if (g[k] > 0) {
            // Only maximize production for (positively) defined and activated 
            // desire classes.
            au.push_back(solver.MakeProd(f[k], 
                                         solver.MakeScalProd(a, u_k))->Var());
        }

        // Add ">=" constraints for positive goals, as we don't mind if we
        // get higher utility than required.
        // Negative (unspecified) goal values follow the same rule, as they
        // might get produced if they're part of a strategy's dependencies
        // (R matrix).
        // However, add "=" constraints for zero goals with u_ik in the sum, as 
        // we don't want to produce utility for specifically unwanted desires, 
        // even if it's only through a dependency.
        if (g[k] != 0) {
            // NOTE: We need to saturate g_k at zero.
            // Otherwise, some requirements might get ignored when (u_ik - r_ik)
            // ends with a negative value.
            Scalar g_k = (g[k] < 0) ? 0 : g[k];

            or_tools::IntExpr* scal_prod = solver.MakeScalProd(a, ur_k);
            or_tools::IntExpr* fk_gk     = solver.MakeProd(f[k], g_k);
            solver.AddConstraint(solver.MakeGreaterOrEqual(
                scal_prod->Var(), 
                fk_gk->Var()));

        } else {
            Scalar g_k = 0;
            solver.AddConstraint(
               solver.MakeScalProdEquality(
                   a,
                   u_k,
                   g_k));
            solver.AddConstraint(
               solver.MakeScalProdEquality(
                   a,
                   r_k,
                   g_k));
        }
    }

    // Maximize the number of activated desire classes based on intensity 
    // (f * s) and/or total raw utility production (p).
    or_tools::IntVar*      f_s       = solver.MakeScalProd(f, s)->Var();
    or_tools::IntVar*      p         = solver.MakeSum(au)->Var(); 

    or_tools::IntVar* opt_sum = params.max_p ? solver.MakeSum(f_s, p)->Var() : 
                                               f_s;

    or_tools::OptimizeVar* opt_var = solver.MakeMaximize(opt_sum, 1);

    // Build a single vector for all variables to solve (a and f vectors):
    SolverImpl::IntVarVector full_sol(nb_strats + nb_cls);
    std::copy(a.begin(), a.end(), full_sol.begin());
    std::copy(f.begin(), f.end(), &full_sol[nb_strats]);

    // Choose a random unbound variable to start, start with minimum values for
    // all (everything deactivated).
    or_tools::DecisionBuilder* const db = solver.MakePhase(
       full_sol, 
       or_tools::Solver::CHOOSE_FIRST_UNBOUND,
       or_tools::Solver::ASSIGN_MIN_VALUE);

    or_tools::SolutionCollector* coll = 
        solver.MakeBestValueSolutionCollector(true);


    coll->Add(full_sol);
    coll->AddObjective(opt_sum);

    std::vector<or_tools::SearchMonitor*> monitors;
    monitors.push_back(coll);
    
    if (params.log) {
        // Full search log monitors, output to stdout. 
        monitors.push_back(solver.MakeSearchLog(100000));
    }

    if (params.time_limit > 0) {
        // Search time limit, in ms.
        monitors.push_back(solver.MakeTimeLimit(params.time_limit));
    }

    if (params.sa) {
        // TODO: Find actual documentation on this / way to monitor its effect, 
        // not available in the official one.
        monitors.push_back(solver.MakeSimulatedAnnealing(true,
                                                         opt_sum,
                                                         1,
                                                         100));
    }

    ros::Time search_start = ros::Time::now();

    solver.NewSearch(db, monitors);
    solver.Solve(db, coll, opt_var);

    ros::Duration search_time = ros::Time::now() - search_start;
    ROS_DEBUG("Solving search duration: %f s", search_time.toSec());

    int nb_sols = coll->solution_count();
    ROS_DEBUG("Solutions count: %i", nb_sols);

    if (nb_sols == 0) {
        ROS_ERROR("IW Solver could not find a solution.");
        a_res_.clear();
    } else {
        a_res_.resize(a.size());
        for (size_t i = 0; i < a_res_.size(); ++i) {
            int64 a_i = coll->Value(0, a[i]);
            a_res_[i] = (a_i > 0);
        }
    }

    std::stringstream ss;
    ss << "[";
    for (size_t k = 0; k < nb_cls; ++k) {
        int64 f_k = coll->Value(0, f[k]);
        ss << f_k << " ";
    }
    ss << "]";
    ROS_DEBUG("F vector: %s", ss.str().c_str());

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

