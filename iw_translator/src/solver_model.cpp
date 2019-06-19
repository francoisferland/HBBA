#include <iw_translator/solver_model.hpp>

using namespace iw_translator;

SolverModel::SolverModel()
{
}

SolverModel::SolverModel(
    const Matrix& u,
    const Matrix& c,
    const Matrix& r,
    const Vector& m):
    u_(u),
    c_(c),
    r_(r),
    m_(m)
{
    updateUR();
}

void SolverModel::updateUR()
{
    // Copy the utility matrix to get proper size.
    ur_ = u_;
    
    // Combined utility matrix (U - R) generation:
    for (size_t i = 0; i < ur_.shape()[0]; ++i) {
        for (size_t k = 0; k < ur_.shape()[1]; ++k) {
            ur_[i][k] = u_[i][k] - r_[i][k];
        }
    }
}

