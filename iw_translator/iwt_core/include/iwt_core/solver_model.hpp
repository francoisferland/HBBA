#ifndef SOLVER_MODEL_HPP
#define SOLVER_MODEL_HPP

#include <boost/multi_array.hpp>
#include <boost/bimap.hpp>

namespace iw_translator
{

    /// \brief Scalar type for internal data (int).
    typedef int                                   Scalar;
    /// \brief Vector type for internal data (Nx1).
    typedef std::vector<Scalar>                   Vector;
    /// \brief Matrix type for internal data (NxM).
    typedef boost::multi_array<Scalar, 2>         Matrix;
    /// \brief Column view of matrices.
    typedef Matrix::const_array_view<1>::type     MatrixColView;
    /// \brief Range generator for matrices.
    typedef boost::multi_array_types::index_range MatrixRange;


    /// \brief A class that represents the static data part of the IW solver.
    ///
    /// Entirely baseed on the available strategies (U, C, and R matrices) and 
    /// resource capacities (M matrix).
    class SolverModel
    {
    protected:
        Matrix u_;  // Utility of strategies (u_ik).
        Matrix c_;  // Costs of strategies (c_ij).
        Matrix r_;  // Utility requirements of strategies (r_ik). 
        Matrix ur_; // The combined utility of strategies (ur_ik).
        Vector m_;  // Resource capacities (m_j).


    public:
        /// \brief Default constructor.
        ///
        /// Produces an empty model.
        ///
        SolverModel(); 

        /// \brief Raw data constructor.
        ///
        /// Builds the model with pre-filled matrices.
        SolverModel(
            const Matrix& u,
            const Matrix& c,
            const Matrix& r,
            const Vector& m);

        const Matrix& u() const { return u_; }
        const Matrix& c() const { return c_; }
        const Matrix& r() const { return r_; }
        const Vector& m() const { return m_; }

        /// \brief Return the combined matrix of produced and required utility
        /// of strategies.
        ///
        /// Obtained with U - R, used for the internal solver as this part is
        /// fixed with each pass.
        const Matrix& ur() const { return ur_; }

    protected:
        /// \brief Recalculates the UR matrix based on U and R.
        void updateUR();
    private:
        /// \brief Disabled copy constructor.
        ///
        /// Needs a special definition since Matrix objects cannot be simply
        /// assigned - sizes need to match, which isn't the case with a
        /// default-constructed SolverModel.
        ///
        SolverModel(const SolverModel& obj);

    };

}

#endif

