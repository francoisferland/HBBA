#ifndef SOLVER_MODEL_HPP
#define SOLVER_MODEL_HPP

#include <hbba_msgs/Strategy.h>
#include <hbba_msgs/DesiresSet.h>
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
    /// \brief Map used to match string identifiers to indices.
    typedef boost::bimap<std::string, int>        IndicesMap;
    /// \brief Column view of matrices.
    typedef Matrix::const_array_view<1>::type     MatrixColView;
    /// \brief Range generator for matrices.
    typedef boost::multi_array_types::index_range MatrixRange;

    /// \brief A class that represents the static data part of the IW solver.
    ///
    /// Entirely baseed on the available strategies (U, C, and R matrices) and 
    /// resource capacities (M matrix).
    /// Can convert desires set to a minimum required utility and intensity 
    /// vectors (G and S).
    /// Also provides methods to match string identifiers to indices for 
    /// strategies (i), resources (j), and desire classes (k).
    ///
    class SolverModel
    {
    private:
        IndicesMap strat_map_; // Strategies (i)
        IndicesMap res_map_;   // Resources  (j)
        IndicesMap cls_map_;   // Classes    (k)

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

        /// \brief Constructor.
        ///
        /// Takes a set of strategies and resources capacities and build the
        /// model.
        /// The strategy indices are distributed in the same order that it was
        /// given in the strats parameter.
        /// In other words, The first strategy in the strats vector will
        /// be at row i=0 of the U, C and R matrices.
        /// The same cannot be guaranteed for the resource indices, as they are
        /// distributed on a first-seen basis.
        ///
        /// The given data need to match between both parameters, or the model
        /// might turn unsolvable.
        /// Consistency issues are warned to the user through ROS_WARN.
        /// 
        /// \param strats A vector of strategies.
        /// \param caps   A vector of resource capacities.
        ///
        SolverModel(
            const std::vector<hbba_msgs::Strategy>&      strats,
            const std::vector<hbba_msgs::ResourceUsage>& caps);

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

        /// \brief Convert a desires set into a minimum required utility vector
        /// (G).
        ///
        /// Entirely based on the declared utility classes.
        /// Indicate through ROS_WARN when unknown classes are encountered.
        /// The whole desires vector is always verified so that every unknown
        /// classes can be warned to the user.
        ///
        /// The default utility value is a negative integer for unspecified 
        /// classes.
        /// A value of 0 is allowed, and is used by desires to prevent utility
        /// production in specific classes.
        ///
        /// The default intensity value is 0.
        ///
        /// \param  desires The input desires set.
        /// \param  g       The output vector for utility requirements.
        ///                 Will be resized and cleared first.
        /// \param  s       The output vector for intensity.
        ///                 Will be resized and cleared first.
        /// \return False when a desire class could not be found in the model.
        bool convertDesires(
            const hbba_msgs::DesiresSet& desires, 
            Vector& g,
            Vector& s) const;

        /// \brief Return the name of strategy with the given index.
        ///
        /// \return NULL_ID in case of unknown indices.
        const std::string& strategyId(int i) const;

        /// \brief Produce the utility (U) matrix as a CSV table.
        ///
        /// \param out The string that will receive the CSV output.
        std::string uAsCSV() const;

        /// \brief Produce the cost matrix (C) matrix as a CSV table.
        ///
        /// \param out The string that will receive the CSV output.
        std::string cAsCSV() const;

        /// \brief Produce the utility requirements (R) matrix as a CSV table.
        ///
        /// \param out The string that will receive the CSV output.
        std::string rAsCSV() const;

        /// \brief Produce the combined utility matrix (UR) as a CSV table.
        ///
        /// \param out The string that will receive the CSV output.
        std::string urAsCSV() const;

    private:
        /// \brief Disabled copy constructor.
        ///
        /// Needs a special definition since Matrix objects cannot be simply
        /// assigned - sizes need to match, which isn't the case with a
        /// default-constructed SolverModel.
        ///
        SolverModel(const SolverModel& obj);

        std::string matrixAsCSV(const Matrix& mtx, const IndicesMap& map) const;

    };

}

#endif

